import socket, subprocess, os, struct, time, select, sys

def fcs(data):
    value=0xffff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value=(value>>1)^ (0x8408 if value&1 else 0)
    return value

left,right=socket.socketpair()
os.dup2(right.fileno(),45)
os.set_inheritable(45,True)
launcher=os.path.join(os.path.dirname(__file__),'..','scripts','ppp-slirp.py')
child=subprocess.Popen([sys.executable,launcher],pass_fds=(45,))
right.close(); os.close(45)
buffer=bytearray(); escaped=False
def send(proto,payload):
    data=b'\xff\x03'+struct.pack('!H',proto)+payload
    data+=struct.pack('<H',fcs(data)^0xffff)
    wire=bytearray(b'\x7e')
    for b in data:
        if b<32 or b in (0x7d,0x7e): wire.extend((0x7d,b^32))
        else: wire.append(b)
    wire.append(0x7e); left.sendall(wire)
def ctrl(proto,code,ident,data=b''):
    send(proto,struct.pack('!BBH',code,ident,len(data)+4)+data)
def read(timeout=1):
    global escaped
    deadline=time.time()+timeout
    while time.time()<deadline:
        if not select.select([left],[],[],max(0,deadline-time.time()))[0]: return
        chunk=left.recv(1)
        if not chunk: raise RuntimeError('PPP backend closed')
        b=chunk[0]
        if b==0x7e:
            data=bytes(buffer); buffer.clear(); escaped=False
            if len(data)>3 and fcs(data)==0xf0b8:
                data=data[:-2]
                if data.startswith(b'\xff\x03'): data=data[2:]
                if data[0]&1: return data[0],data[1:]
                return struct.unpack('!H',data[:2])[0],data[2:]
        elif b==0x7d: escaped=True
        else:
            buffer.append(b^32 if escaped else b); escaped=False

def negotiate(proto,ident,options):
    deadline=time.time()+20; last=0
    while time.time()<deadline:
        if time.time()-last>2:
            ctrl(proto,1,ident,options); last=time.time()
        packet=read()
        if not packet: continue
        p,data=packet
        if p in (0xc021,0x8021):
            code,i,n=struct.unpack('!BBH',data[:4]); opts=data[4:n]
            print('control',hex(p),code,i,opts.hex(),flush=True)
            if code==1: ctrl(p,2,i,opts)
            if p==proto and code==2 and i==ident and opts==options: return
            if p==proto and code==3: options=opts; ident+=1; last=0
    raise RuntimeError('Negotiation timeout')
try:
    negotiate(0xc021,10,b'')
    negotiate(0x8021,20,b'\x03\x06'+socket.inet_aton('10.0.2.15'))
    print('LCP + IPCP negotiated',flush=True)
    query=b'\x90\x42\x01\x00\x00\x01\x00\x00\x00\x00\x00\x00'+b'\x07example\x03com\x00\x00\x01\x00\x01'
    udp=struct.pack('!HHHH',12345,53,8+len(query),0)+query
    header=struct.pack('!BBHHHBBH4s4s',0x45,0,20+len(udp),1,0,64,17,0,socket.inet_aton('10.0.2.15'),socket.inet_aton('10.0.2.3'))
    words=struct.unpack('!10H',header); total=sum(words)
    while total>>16: total=(total&65535)+(total>>16)
    header=header[:10]+struct.pack('!H',total^65535)+header[12:]
    send(0x21,header+udp)
    deadline=time.time()+15
    while time.time()<deadline:
        packet=read()
        if packet and packet[0]==0x21:
            ip=packet[1]; ihl=(ip[0]&15)*4
            if ip[9]==17 and ip[ihl+8:ihl+10]==b'\x90\x42':
                assert ip[ihl+11]&15==0, 'DNS error'
                print('PASS: DNS response crossed PPP userspace routing',flush=True)
                break
    else: raise RuntimeError('DNS response timeout')
finally:
    left.close(); child.terminate(); child.wait(timeout=5)
