import time
import serial
import struct
#import calendar
import datetime

sn = 0            #Адрес счётчика, если счётчик один то можно оставить 0
koof = 1.0848     #Коофициент потерь для НВК 1.0848

def COM_Init():
  global ser
  # configure the serial connections (the parameters differs on the device you are connecting to)
  ser = serial.Serial()
  ser.port = 'COM10'
  ser.baudrate = 9600
  ser.bytesize = serial.EIGHTBITS #number of bits per bytes
  ser.parity = serial.PARITY_NONE #set parity check: no parity
  ser.stopbits = serial.STOPBITS_ONE #number of stop bits
  ser.timeout = 0.1            #non-block read
  ser.xon  = False
  ser.xoff = False     #disable software flow control
  ser.open()
  ser.write(pack_data(b'\x01\x01\x01\x01\x01\x01\x01\x01'))
  time.sleep(0.5)
  while ser.inWaiting() > 0:
    out = ser.read(ser.inWaiting()) 

def crc16(data):
    crc = 0xFFFF 
    l = len(data)
    i = 0
    while i < l:
        j = 0
        crc = crc ^ data[i]
        while j < 8:
            if (crc & 0x1):
                mask = 0xA001
            else:
                mask = 0x00
            crc = ((crc >> 1) & 0x7FFF) ^ mask
            j += 1
        i += 1
    if crc < 0:
        crc -= 256
        
    result = data + chr(crc % 256).encode('latin-1') + chr(crc // 256).encode('latin-1')
    return result

def pack_data(data):
  chunk = struct.pack('B', int(sn))
  chunk += data
  chunk = crc16(chunk)  
  return chunk
  
COM_Init()

ser.write(pack_data(b'\x08\x13')) 
time.sleep(0.5)        
while ser.inWaiting() > 0:
  out = ser.read(ser.inWaiting()) 
#print(out)
adr = int('{:02x}'.format(out[1])+'{:02x}'.format(out[2]),16)
tt = '{:02x}'.format(out[4])+':'+'{:02x}'.format(out[5])
dd = datetime.date(int('20'+'{:02x}'.format(out[8])),int('{:02x}'.format(out[7])),int('{:02x}'.format(out[6])))
dl = str(out[9])

if '{:02x}'.format(out[5])=='30':
  tmp_h = int('{:02x}'.format(out[4]))*32
else:
  tmp_h = int('{:02x}'.format(out[4]))*32-16

if dd.month == 1:
  tmp_year = dd.year - 1
  tmp_month = 12
else:
  tmp_year = dd.year
  tmp_month = dd.month - 1
  
dd_first = datetime.date(tmp_year,tmp_month,1) # Первый день предыдущего месяца
dd_last = datetime.date(dd.year,dd.month,1)    # Первый день текущего месяца
mday =  dd_last - dd_first                     # Кол-во дней в месяце
hours = mday.days*24
tmp_mday = dd - dd_first
adr_first = adr - (tmp_mday.days*768+tmp_h)
if adr_first < 0:
  adr_first = 65536 + adr_first

print ('ADR: '+hex(adr_first))
  
print('адрес: '+str(adr)+' время: '+tt+' дата: '+str(dd.year)+' длительность: '+dl)

f = open(str(dd_first.year)+'-'+str(dd_first.month)+'.csv','w')
sutki = 1

for i in range(hours):   #hours
  pack_d = b'\x06\x03'
  adr_hex = '{:04x}'.format(adr_first)
  #print(adr_hex[0:2])
  #print(adr_hex[2:4])
  pack_d += struct.pack('B', int(adr_hex[0:2],16))
  pack_d += struct.pack('B', int(adr_hex[2:4],16))
  pack_d += struct.pack('B', int('1e',16))

  ser.write(pack_data(pack_d)) 
  time.sleep(0.5)        
  while ser.inWaiting() > 0:
    out = ser.read(ser.inWaiting()) 

  pw = ((int('{:02x}'.format(out[9])+'{:02x}'.format(out[8]),16)+ int('{:02x}'.format(out[24])+'{:02x}'.format(out[23]),16))/2000)*koof

  f.write(str(pw).replace('.',',')+';')
  if sutki==24:
    sutki = 0
    f.write('\n')
    
  sutki += 1
  #print('{:02x}'.format(out[2])+':'+'{:02x}'.format(out[3])+'\t'+'20'+'{:02x}'.format(out[6])+'-'+'{:02x}'.format(out[5])+'-'+'{:02x}'.format(out[4])+'\t'+str(pw))

  print('{:02x}'.format(out[17])+':'+'{:02x}'.format(out[18])+'\t'+'20'+'{:02x}'.format(out[21])+'-'+'{:02x}'.format(out[20])+'-'+'{:02x}'.format(out[19])+'\t'+str(pw))
  adr_first = adr_first + 32
  if adr_first > 65535:
    adr_first = adr_first - 65536


f.close()
ser.close()
exit()  
  
  
