import struct
import numpy as np, random, sys
from PIL import Image, ImageDraw, ImageFilter
sys.path.insert(0,'/home/claude/work/tex')
src=Image.open('/tmp/milkleak.png').convert('RGBA')
A0=np.array(src.resize((2048,2048),Image.BOX))[...,3].astype(np.float32)/255
H,W=A0.shape
# reuse dxt5 from old generator
code=open('/home/claude/work/tex/gen_stages2.py').read()
start=code.index('def dxt5'); end=code.index('# n streams')
exec(code[start:end])
CX=1024.0; CROTCH_Y=1968.0
centres=[(1717.8/2,2896.3/2),(2376.9/2,2895.2/2)]
def seam(x): return 1995.0-abs(x-CX)*0.75
def sstep(a,b,v):
    t=min(1,max(0,(v-a)/(b-a))); return t*t*(3-2*t)
class Noise:
    def __init__(s,r): s.p=[(r.uniform(0.008,0.03),r.uniform(0,6.28),r.uniform(0.5,2.2)) for _ in range(4)]
    def __call__(s,y): return sum(a*np.sin(f*y+p) for f,p,a in s.p)
# stage: wet streams count, lengths, width, how many of the inner streams follow the belly to the vulva
stages=[
 dict(n=2, L=(30,70),  w=(2.5,4), sheen=0.20,boost=1.5,reach=0,dried=0, sheet=0,  belly=0, drip=0.0, seed=1),
 dict(n=3, L=(50,120), w=(3,4.5), sheen=0.30,boost=1.8,reach=0,dried=0, sheet=0,  belly=0, drip=0.0, seed=2),
 dict(n=5, L=(90,190), w=(3,5),   sheen=0.42,boost=2.0,reach=0,dried=1, sheet=0,  belly=1, drip=0.15,seed=3),
 dict(n=7, L=(130,270),w=(3.5,6), sheen=0.55,boost=2.3,reach=0,dried=2, sheet=0,  belly=2, drip=0.25,seed=4),
 dict(n=9, L=(170,340),w=(4,7),   sheen=0.68,boost=2.6,reach=1,dried=2, sheet=0.3,belly=3, drip=0.3, seed=5),
 dict(n=12,L=(210,400),w=(4.5,8), sheen=0.80,boost=2.9,reach=2,dried=3, sheet=0.5,belly=5, drip=0.35,seed=6),
 dict(n=16,L=(250,460),w=(5,10),  sheen=0.92,boost=3.2,reach=4,dried=3, sheet=0.8,belly=7, drip=0.4, seed=7),
 dict(n=22,L=(280,500),w=(6,12),  sheen=1.00,boost=3.6,reach=6,dried=4, sheet=1.0,belly=9, drip=0.45,seed=8),
]
UNDER=None
def path(rng,x,y,pull,length,reach,wet_rows,drip=0.0,cy=0.0):
    """Returns segments (lists of points). Milk first runs over the curve of the breast (starts at an angle and
    is bent down by gravity), may drip off the underside onto the belly, then runs down the belly."""
    nz=Noise(rng); pts=[(x,y)]; segs=[pts]; t=0
    side=-1 if x<CX else 1
    end_x=CX+side*rng.uniform(2,20); gap=rng.uniform(24,150)
    lazy=rng.uniform(0.025,0.07)
    cdx=rng.uniform(0.45,0.85); cdy=np.sqrt(1-cdx*cdx)
    th0=rng.gauss(0,0.45)            # direction the milk leaves the nipple over the breast curve
    lam=rng.uniform(30,90)                # how quickly gravity bends it straight down
    under=cy+rng.uniform(85,125)           # underside of the breast
    dripped=False; will_drip=rng.random()<drip
    dx,dy=np.sin(th0),np.cos(th0)
    while True:
        if will_drip and not dripped and y>=under:
            # falls off the underside of the breast and lands lower on the belly
            dripped=True
            y+=rng.uniform(18,70); x+=rng.uniform(-6,6)
            pts=[(x,y)]; segs.append(pts); dx,dy=0.0,1.0
            continue
        if reach:
            if (x-CX)*side<0:            # met the middle line early: runs straight down it
                x=CX+side*0.5; dx,dy=0.0,1.0
            target_y=seam(x)-gap
            a=sstep(target_y-160,target_y,y)
            wx,wy=-side*cdx*a,(1-a)+a*cdy
            if a>0.95: wy+=(seam(x)-26-y)*0.004
            n=np.hypot(wx,wy); wx,wy=wx/n,wy/n
            k=lazy if t>lam else max(lazy,1.5/lam)
            dx+=(wx-dx)*k; dy+=(wy-dy)*k
            n=np.hypot(dx,dy); dx,dy=dx/n,dy/n
            m=(nz(t*1.8)-nz(t*1.8-2.7))*3.2; t+=1.5
            x+=dx*1.5+dy*m
            y+=dy*1.5-dx*m*0.5
            if y>=seam(x)-24:
                y=seam(x)-24
                dx+=(-side*0.8-dx)*0.15; dy+=(0.6-dy)*0.15
                n=np.hypot(dx,dy); dx,dy=dx/n,dy/n
            if (abs(x-CX)<=abs(end_x-CX) and y>seam(x)-70) or y>=CROTCH_Y or t>3000: break
        else:
            th=th0*np.exp(-t/lam) if not dripped else 0.0
            sx,sy=np.sin(th),np.cos(th)
            t+=1.5
            belly=sstep(1700,1950,y)
            x+=sx*1.5-pull*belly*(x-CX)*0.004+(nz(y)-nz(y-1.5))*1.2
            y+=sy*1.5
            if t>=length or y>=seam(x)-24: break
        row=wet_rows.get(int(y)//3)
        if row:
            near=min(row,key=lambda v:abs(v-x))
            if abs(near-x)<5: x+=(near-x)*0.03
        pts.append((x,y))
    for seg in segs:
        for (px,py) in seg: wet_rows.setdefault(int(py)//3,[]).append(px)
    return [sg for sg in segs if len(sg)>1]
def draw_seg(img,pts,w0,w1,val,rng):
    d=ImageDraw.Draw(img); n=len(pts); nz=Noise(rng)
    bulges=set(rng.sample(range(n),min(n//220,3))) if n>220 else set()
    for i,(x,y) in enumerate(pts):
        t=i/max(n-1,1); r=max(0.7,(w0+(w1-w0)*t)/2)*(1+0.12*nz(i*1.3))
        d.ellipse([x-r,y-r,x+r,y+r],fill=val)
        if i in bulges:
            b=r*rng.uniform(1.15,1.45); d.ellipse([x-b,y-b*0.8,x+b,y+b*1.3],fill=val)
def splat(img,x,y,r,val,rng):
    d=ImageDraw.Draw(img)
    d.ellipse([x-r,y-r*0.8,x+r,y+r*0.9],fill=val)
    for j in range(rng.randint(1,4)):            # little spatters around where the drop landed
        a=rng.uniform(0,6.28); dist=r*rng.uniform(1.1,2.2); rr=r*rng.uniform(0.15,0.4)
        px,py=x+np.cos(a)*dist,y+np.sin(a)*dist*0.7
        d.ellipse([px-rr,py-rr,px+rr,py+rr],fill=int(val*0.9))
def draw_trail(img,segs,w0,w1,val,rng=random.Random(5)):
    for k,seg in enumerate(segs):
        if k>0:
            splat(img,seg[0][0],seg[0][1],w0*rng.uniform(0.9,1.4),val,rng)
            w0=w0*0.85
        draw_seg(img,seg,w0,w1,val,rng)
for idx,st in enumerate(stages,1):
    rng=random.Random(st['seed']*31+7)
    wet=Image.new('L',(W,H),0); dry=Image.new('L',(W,H),0); beads=Image.new('L',(W,H),0); sheen=Image.new('L',(W,H),0)
    wd=ImageDraw.Draw(wet); bd=ImageDraw.Draw(beads); sd=ImageDraw.Draw(sheen); sheet=Image.new('L',(W,H),0)
    wet_rows={}
    for side,(cx,cy) in enumerate(centres):
        s=st['sheen']
        for r,a in [(30,40),(20,90),(11,150)]:
            rr=r*(0.7+0.5*s); wd.ellipse([cx-rr,cy-rr*0.8,cx+rr,cy+rr*0.8],fill=int(a*s+40*(1-s)))
        if idx>=6:   # wet film over the lower breast
            for j in range(6):
                rr=rng.uniform(32,60)*(0.8+0.15*(idx-6)); ox=rng.uniform(-22,22); oy=rng.uniform(5,50)
                sd.ellipse([cx+ox-rr,cy+oy-rr*0.6,cx+ox+rr,cy+oy+rr*0.9],fill=255)
        inward=1 if side==0 else -1      # +x toward the body centre for the left nipple
        nreach=(st['reach']+(1 if side==0 else 0))//2
        # 1) streams that follow the belly down to the vulva: start on the inner side of the nipple
        for k in range(nreach):
            x=cx+rng.uniform(-14,16)*inward; y=cy+rng.uniform(0,6)
            w=rng.uniform(*st['w'])*(1.0 if k<2 else rng.uniform(0.5,0.8))
            pts=path(rng,x,y,rng.uniform(0.85,1.15),0,True,wet_rows,0.0,cy)
            draw_trail(wet,pts,w,w*rng.uniform(0.85,1.0),228,rng)
            if st['sheet'] and k<2:
                sw=w*rng.uniform(3.5,6.0)*st['sheet']
                draw_trail(sheet,[pts[0][len(pts[0])//8:]],sw,sw*0.5,255,rng)
        # 2) wet streams that are still running (shorter, end in a hanging drop)
        nwet=st['n']-nreach-st['dried']
        for k in range(max(nwet,1)):
            x=cx+rng.uniform(-20,20)*(0.6+0.5*s); y=cy+rng.uniform(-2,6)
            w=rng.uniform(*st['w'])*rng.uniform(0.6,1.0)
            L=rng.uniform(*st['L'])*rng.uniform(0.5,1.0)
            pts=path(rng,x,y,rng.uniform(0.6,1.0),L,False,wet_rows,st['drip'],cy)
            draw_trail(wet,pts,w,w*0.6,225,rng)
            ex,ey=pts[-1][-1]; rr=w*rng.uniform(0.6,0.9)
            bd.ellipse([ex-rr,ey-rr*0.7,ex+rr,ey+rr*1.3],fill=240)
        # 3) streams that ran toward the legs earlier, stopped, pooled into a bead and dried: faint trail + bead ring
        for k in range(st['dried']):
            x=cx-inward*rng.uniform(0,22)+rng.uniform(-6,6); y=cy+rng.uniform(0,8)
            w=rng.uniform(*st['w'])*rng.uniform(0.5,0.8)
            L=rng.uniform(st['L'][0]*0.9,st['L'][1]*1.15)
            pts=path(rng,x,y,rng.uniform(0.2,0.5),L,False,wet_rows,st['drip']*0.5,cy)
            # dried residue: thin, patchy
            d=ImageDraw.Draw(dry); flat=[p for sg in pts for p in sg]; n=len(flat)
            for sg in pts[1:]:
                splat(dry,sg[0][0],sg[0][1],w*0.9,170,rng)
            pts=flat
            for i,(px,py) in enumerate(pts):
                t=i/max(n-1,1); r=max(0.6,w*(0.55-0.2*t)/2)
                v=int(150*(0.75+0.25*np.sin(py*0.11+k)) * (0.7+0.3*rng.random()))
                d.ellipse([px-r,py-r,px+r,py+r],fill=v)
            ex,ey=pts[-1]; rr=w*rng.uniform(0.9,1.3)
            # dried bead: opaque rim, thinner middle
            d.ellipse([ex-rr,ey-rr*0.75,ex+rr,ey+rr*1.2],fill=205)
            d.ellipse([ex-rr*0.55,ey-rr*0.35,ex+rr*0.55,ey+rr*0.75],fill=150)
    # milk that dripped off the breasts onto the belly: a splat where it landed and a short run down (some dried)
    for side,(cx,cy) in enumerate(centres):
        for k in range((st['belly']+side)//2):
            x=cx+rng.uniform(-75,75); y=cy+rng.uniform(95,135)+rng.uniform(25,300)
            if y>seam(x)-70: y=seam(x)-70-rng.uniform(0,60)
            dried=rng.random()<0.45
            w=rng.uniform(*st['w'])*rng.uniform(0.55,0.95)
            L=rng.uniform(6,30)
            nzb=Noise(rng); seg=[(x,y)]; px,py=x,y; tt=0
            while tt<L and py<seam(px)-28:
                tt+=1.5; py+=1.5; px+=(nzb(py)-nzb(py-1.5))*1.1-(px-CX)*0.0012*sstep(1700,1950,py)
                seg.append((px,py))
            if dried:
                splat(dry,x,y,w*rng.uniform(0.9,1.5),175,rng)
                dd=ImageDraw.Draw(dry)
                for i,(qx,qy) in enumerate(seg):
                    r=max(0.6,w*(0.5-0.25*i/max(len(seg),1))/2)
                    dd.ellipse([qx-r,qy-r,qx+r,qy+r],fill=int(140*(0.7+0.3*rng.random())))
                ex,ey=seg[-1]; rr=w*0.9
                dd.ellipse([ex-rr,ey-rr*0.75,ex+rr,ey+rr*1.2],fill=200)
                dd.ellipse([ex-rr*0.5,ey-rr*0.3,ex+rr*0.5,ey+rr*0.7],fill=145)
            else:
                splat(wet,x,y,w*rng.uniform(0.8,1.2),228,rng)
                if len(seg)>1: draw_seg(wet,seg,w*1.3,w*0.3,225,rng)
                if len(seg)>40:
                    ex,ey=seg[-1]; rr=w*0.55
                    bd.ellipse([ex-rr,ey-rr*0.7,ex+rr,ey+rr*1.3],fill=232)
    # streams running across the belly: milk that came off the underside of the breasts and keeps running down the abdomen
    rivers=[0,0,0,1,2,3,4,6][idx-1]
    for side,(cx,cy) in enumerate(centres):
        inward=1 if side==0 else -1
        for k in range((rivers+side)//2+ (1 if rivers and side==0 else 0)):
            x=cx+rng.uniform(25,120)*inward; y=cy+rng.uniform(105,150)
            w=rng.uniform(*st['w'])*rng.uniform(0.6,0.95)
            L=rng.uniform(120,240+40*idx)
            nzr=Noise(rng); seg=[(x,y)]; px,py=x,y; tt=0; drift=rng.uniform(0.0,0.35)*inward
            while tt<L and py<seam(px)-30:
                tt+=1.5; py+=1.5
                # the belly curves away to the sides, so milk slowly gathers toward the middle line as it runs down
                px+=drift*sstep(cy+100,cy+400,py)+(nzr(py*1.4)-nzr(py*1.4-2.1))*2.2-(px-CX)*0.0018*sstep(1650,1950,py)
                seg.append((px,py))
            splat(wet,x,y,w*rng.uniform(0.55,0.75),215,rng)
            if rng.random()<0.25 and idx<8:   # this one has dried
                dd=ImageDraw.Draw(dry)
                for i,(qx,qy) in enumerate(seg):
                    r=max(0.6,w*(0.6-0.3*i/len(seg))/2)
                    dd.ellipse([qx-r,qy-r,qx+r,qy+r],fill=int(145*(0.7+0.3*rng.random())))
                ex,ey=seg[-1]; rr=w*0.9
                dd.ellipse([ex-rr,ey-rr*0.75,ex+rr,ey+rr*1.2],fill=200); dd.ellipse([ex-rr*0.5,ey-rr*0.3,ex+rr*0.5,ey+rr*0.7],fill=145)
            else:
                draw_seg(wet,seg,w,w*0.55,224,rng)
                ex,ey=seg[-1]; rr=w*0.7
                bd.ellipse([ex-rr,ey-rr*0.7,ex+rr,ey+rr*1.3],fill=236)
            if st['sheet']>0 and k==0:
                draw_trail(sheet,[seg],w*3.0*st['sheet'],w*1.5*st['sheet'],255,rng)
    # above the nipples: spray that landed higher up on the breast runs back down toward the nipple
    above=[0,1,1,2,3,4,5,6][idx-1]
    for side,(cx,cy) in enumerate(centres):
        for k in range(above):
            ox=rng.uniform(-30,30); start=rng.uniform(18,45+8*idx)
            x=cx+ox; y=cy-start
            w=rng.uniform(st['w'][0]*0.45,st['w'][1]*0.5)
            # fine spray specks that landed there (most stay as specks)
            for j in range(rng.randint(2,5)):
                r=w*rng.uniform(0.2,0.45); qx=x+rng.uniform(-14,14); qy=y+rng.uniform(-12,10)
                bd.ellipse([qx-r,qy-r,qx+r,qy+r*1.15],fill=210)
            seg=[(x,y)]; px,py=x,y; nzr=Noise(rng)
            stop=cy-rng.uniform(-6,14)               # most reach the areola, a few stop just short
            lean=rng.uniform(-0.25,0.25)
            while py<stop:
                py+=1.2
                # runs down over the curve of the breast and is led toward the nipple
                px+=lean*(1-sstep(cy-start,cy,py))+(cx-px)*0.015*sstep(cy-start,cy,py)+(nzr(py*2.2)-nzr(py*2.2-2.6))*1.4
                seg.append((px,py))
            if len(seg)>2:
                # thin where it started, fuller as it gathers
                draw_seg(wet,seg,w*0.35,w*0.85,212,rng)
        if idx>=6:   # shine over the upper half of the breast, where the spray landed
            for j in range(2+idx-6):
                rr=rng.uniform(16,30); ox=rng.uniform(-25,25); oy=-rng.uniform(15,45)
                sd.ellipse([cx+ox-rr,cy+oy-rr*0.8,cx+ox+rr,cy+oy+rr*0.8],fill=170)
    # a damp film on the upper belly at the top stages, where the drops keep landing
    if idx>=6:
        for side,(cx,cy) in enumerate(centres):
            for j in range(3+idx-6):
                rr=rng.uniform(30,55); ox=rng.uniform(-60,60); oy=rng.uniform(120,260)
                sd.ellipse([cx+ox-rr,cy+oy-rr*0.8,cx+ox+rr,cy+oy+rr*1.1],fill=int(150+30*(idx-6)))
    # where the belly streams arrive: a thin wet sheen over the vulva, no hard blob
    if st['reach']:
        k=min(1.0,st['reach']/6)
        for j in range(4):
            rr=rng.uniform(10,20)*(0.6+0.6*k); ox=rng.uniform(-8,8); oy=rng.uniform(-28,-4)
            sd.ellipse([CX+ox-rr*0.7,CROTCH_Y+oy-rr,CX+ox+rr*0.7,CROTCH_Y+oy+rr*0.6],fill=255)
    Wt=np.array(wet.filter(ImageFilter.GaussianBlur(1.2))).astype(np.float32)/255
    Dr=np.array(dry.filter(ImageFilter.GaussianBlur(0.9))).astype(np.float32)/255
    Bd=np.array(beads.filter(ImageFilter.GaussianBlur(1.0))).astype(np.float32)/255
    St=np.array(sheet.filter(ImageFilter.GaussianBlur(14))).astype(np.float32)/255*(0.26+0.14*st['sheet'])
    Halo=np.array(wet.filter(ImageFilter.GaussianBlur(6))).astype(np.float32)/255*0.38
    Sh=np.array(sheen.filter(ImageFilter.GaussianBlur(22))).astype(np.float32)/255*0.45
    out=np.clip(np.maximum.reduce([A0*st['boost'],Wt*0.92,Dr*0.8,Bd*0.95,Sh,St,Halo]),0,1)
    yy,xx=np.mgrid[0:H,0:W]; fall=np.clip((seam(xx)-4-yy)/22.0,0,1)
    out=np.maximum(out*fall,np.minimum(out,A0))
    alpha=(out*255+0.5).astype(np.uint8)
    open(f'MilkLeak_{idx}.dds','wb').write(dxt5(alpha))
    Image.fromarray(alpha[1300:2048,700:1350]).save(f'/tmp/s5_{idx}.png')
ims=[Image.open(f'/tmp/s5_{i}.png').resize((260,280)) for i in range(1,9)]
c=Image.new('RGB',(4*270,2*290),(60,60,60))
for i,im in enumerate(ims): c.paste(im.convert('RGB'),((i%4)*270,(i//4)*290))
c.save('/tmp/s5all.png')
