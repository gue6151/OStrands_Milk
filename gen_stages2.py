import numpy as np, struct, random
from PIL import Image, ImageDraw, ImageFilter
src=Image.open('/tmp/milkleak.png').convert('RGBA')
A0=np.array(src.resize((2048,2048),Image.BOX))[...,3].astype(np.float32)/255
H,W=A0.shape
centres=[(1717.8/2,2896.3/2),(2376.9/2,2895.2/2)]
CX=1024.0          # body centre line
CROTCH=(1024.0,1962.0)
def seam(x):       # approximate groin/leg seam of the CBBE torso island (2048 scale): stay above it
    return 1995.0-abs(x-CX)*0.75
def dxt5(alpha):
    def enc(img):
        h,w=img.shape
        b=img.reshape(h//4,4,w//4,4).transpose(0,2,1,3).reshape(-1,16)
        mx=b.max(1).astype(np.int32); mn=b.min(1).astype(np.int32); same=mx==mn
        r=np.maximum(mx-mn,1)
        t=np.clip(np.rint((mx[:,None]-b.astype(np.int32))*7/r[:,None]).astype(np.int32),0,7)
        code=np.where(t==0,0,np.where(t==7,1,t+1)); code[same]=0
        bits=np.zeros(b.shape[0],np.uint64)
        for i in range(16): bits|=(code[:,i].astype(np.uint64)<<np.uint64(3*i))
        out=np.zeros((b.shape[0],16),np.uint8)
        out[:,0]=mx; out[:,1]=np.where(same,mx,mn)
        for k in range(6): out[:,2+k]=((bits>>np.uint64(8*k))&np.uint64(0xff)).astype(np.uint8)
        out[:,8:12]=0xff
        return out.tobytes()
    data=b''; mips=0; size=W; lvl=Image.fromarray(alpha)
    while size>=4:
        data+=enc(np.array(lvl.resize((size,size),Image.BOX)) if size!=W else alpha); mips+=1; size//=2
    hdr=struct.pack('<4sIIIIIII44x',b'DDS ',124,0x1|0x2|0x4|0x1000|0x20000|0x80000,H,W,W*H,0,mips)
    pf=struct.pack('<II4sIIIII',32,0x4,b'DXT5',0,0,0,0,0)
    caps=struct.pack('<IIII4x',0x1000|0x8|0x400000,0,0,0)
    return hdr+pf+caps+data
# n streams, length, width, sheen, boost, share of streams that run to the crotch, film, crotch pool
stages=[
    dict(n=2, L=(30,70),  w=(2.5,4),  sheen=0.20,boost=1.5,crotch=0.0, film=0,pool=0,seed=1),
    dict(n=3, L=(50,110), w=(3,4.5),  sheen=0.30,boost=1.8,crotch=0.0, film=0,pool=0,seed=2),
    dict(n=4, L=(80,160), w=(3,5),    sheen=0.42,boost=2.0,crotch=0.0, film=0,pool=0,seed=3),
    dict(n=6, L=(110,220),w=(3.5,6),  sheen=0.55,boost=2.3,crotch=0.25,film=0,pool=0,seed=4),
    dict(n=8, L=(150,300),w=(4,7),    sheen=0.68,boost=2.6,crotch=0.4, film=0,pool=0,seed=7),
    dict(n=10,L=(190,380),w=(4.5,8),  sheen=0.80,boost=2.9,crotch=0.55,film=0,pool=0.5,seed=9),
    dict(n=14,L=(240,600),w=(5,10),   sheen=0.92,boost=3.2,crotch=0.75,film=1,pool=0.8,seed=11),
    dict(n=20,L=(300,700),w=(6,12),   sheen=1.00,boost=3.6,crotch=0.9, film=2,pool=1.0,seed=13),
]
def stroke(layer,pts,w0,w1,val):
    d=ImageDraw.Draw(layer)
    n=len(pts)
    for i,(x,y) in enumerate(pts):
        t=i/max(n-1,1); r=max(0.8,(w0+(w1-w0)*t)/2)
        d.ellipse([x-r,y-r,x+r,y+r],fill=val)
for idx,st in enumerate(stages,1):
    rng=random.Random(st['seed'])
    lines=Image.new('L',(W,H),0); film=Image.new('L',(W,H),0); drops=Image.new('L',(W,H),0)
    ld=ImageDraw.Draw(lines); dd=ImageDraw.Draw(drops)
    for cx,cy in centres:
        s=st['sheen']
        for r,a in [(30,40),(20,90),(11,150)]:
            rr=r*(0.7+0.5*s); ld.ellipse([cx-rr,cy-rr*0.8,cx+rr,cy+rr*0.8],fill=int(a*s+40*(1-s)))
        if st['film']:
            fd=ImageDraw.Draw(film)
            for j in range(6):
                rr=rng.uniform(32,60)*(0.85+0.2*st['film']); ox=rng.uniform(-25,25); oy=rng.uniform(-10,45)
                fd.ellipse([cx+ox-rr,cy+oy-rr*0.7,cx+ox+rr,cy+oy+rr],fill=255)
        for k in range(st['n']):
            toCrotch=rng.random()<st['crotch']
            major=k<max(2,st['n']//2) or toCrotch
            x=cx+rng.uniform(-14,14)*(0.6+0.6*s); y=cy+rng.uniform(-3,6)
            width=rng.uniform(*st['w'])*(1 if major else 0.65)
            if toCrotch: width*=0.9
            length=rng.uniform(*st['L'])*(1 if major else 0.5)
            drift=rng.uniform(-0.35,0.35); ph=rng.uniform(0,6.28)
            pts=[(x,y)]; travelled=0.0
            if toCrotch:
                x0,y0=x,y
                tx=CROTCH[0]+rng.uniform(-38,38); ty=CROTCH[1]-(rng.uniform(0,22) if rng.random()<0.6 else rng.uniform(40,200))
                sb=rng.uniform(0.12,0.38)
                p=rng.uniform(1.4,3.0); amp=rng.uniform(2,7); ph=rng.uniform(0,6.28)
                n=int((ty-y0)/2)
                for i in range(1,n+1):
                    t=i/n; u=min(1,max(0,(t-sb)/(1-sb))); e=u*u*(3-2*u); e=e**(p/2)
                    xx=x0+(tx-x0)*e+np.sin(ph+t*rng.uniform(9,13))*amp*(1-t)
                    yy=y0+(ty-y0)*t
                    if yy>=seam(xx)-18: break
                    pts.append((xx,yy))
            else:
              while True:
                x+=drift*1.6+np.sin(ph+y*0.06)*0.8+(CX-x)*0.0015
                if travelled>=length: break
                y+=2; travelled+=2
                if y>=seam(x)-18: break      # never cross onto the leg
                pts.append((x,y))
            stroke(lines,pts,width,width*0.55,230)
            ex,ey=pts[-1]; rr=width*rng.uniform(0.75,1.2)*(0.7 if toCrotch else 1)
            dd.ellipse([ex-rr,ey-rr*0.6,ex+rr,ey+rr*1.25],fill=245)
            if idx>=4 and not toCrotch and rng.random()<0.5:
                for j in range(rng.randint(1,2)):
                    dy=rng.uniform(12,45)*(j+1)
                    if ey+dy < seam(ex)-20:
                        r2=width*rng.uniform(0.45,0.8); dd.ellipse([ex-r2,ey+dy-r2*0.7,ex+r2,ey+dy+r2*1.2],fill=235)
    if st['pool']:
        # milk gathering over the vulva
        fd=ImageDraw.Draw(film)
        for j in range(5):
            rr=rng.uniform(22,40)*(0.7+0.5*st['pool']); ox=rng.uniform(-14,14); oy=rng.uniform(-34,-4)
            fd.ellipse([CROTCH[0]+ox-rr,CROTCH[1]+oy-rr,CROTCH[0]+ox+rr,CROTCH[1]+oy+rr*0.8],fill=255)
    L=np.array(lines.filter(ImageFilter.GaussianBlur(1.3))).astype(np.float32)/255
    D=np.array(drops.filter(ImageFilter.GaussianBlur(1.0))).astype(np.float32)/255
    F=np.array(film.filter(ImageFilter.GaussianBlur(20))).astype(np.float32)/255*(0.42+0.12*max(st['film'],st['pool']))
    out=np.clip(np.maximum.reduce([A0*st['boost'],L*0.92,D*0.95,F]),0,1)
    # hard guard: nothing below the leg seam
    yy,xx=np.mgrid[0:H,0:W]; fall=np.clip((seam(xx)-4-yy)/22.0,0,1)
    out=np.maximum(out*fall,np.minimum(out,A0))
    alpha=(out*255+0.5).astype(np.uint8)
    open(f'MilkLeak_{idx}.dds','wb').write(dxt5(alpha))
    Image.fromarray(alpha[1350:2048,700:1350]).save(f'/tmp/s2_{idx}.png')
    print(idx,(out>0.04).sum())
