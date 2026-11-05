"""Temporary NTP responder; local PC is NTP-synchronised. Robot hosts only."""
import socket,struct,time
ALLOWED={'192.168.234.1','192.168.234.234'}
def stamp(t):
 s=t+2208988800;i=int(s);return struct.pack('!II',i&0xffffffff,int((s-i)*2**32))
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('192.168.234.21',1123))
print('NTP bridge on 192.168.234.21:1123 (robot hosts only)',flush=True)
while True:
 data,addr=s.recvfrom(512);rx=time.time()
 if addr[0] not in ALLOWED or len(data)<48 or data[0]&7!=3:continue
 pkt=bytearray(48);pkt[0]=0x24;pkt[1]=10;pkt[2]=4;pkt[3]=(-20)&255
 struct.pack_into('!II',pkt,4,int(.02*65536),int(.05*65536))
 pkt[12:16]=socket.inet_aton('192.168.234.21');pkt[16:24]=stamp(rx)
 pkt[24:32]=data[40:48];pkt[32:40]=stamp(rx);pkt[40:48]=stamp(time.time())
 s.sendto(pkt,addr)
