# Rasterize the body's UV layout: every texel gets its 3D surface position, normal and island id.
import numpy as np
from scipy.sparse.csgraph import connected_components
from scipy.sparse import coo_matrix
d=np.load('fb0_3BA.npz'); pos,uv,tris=d['pos'].astype(np.float64),d['uv'].astype(np.float64),d['tris'].astype(np.int64)
S=2048
# vertex normals
fn=np.cross(pos[tris[:,1]]-pos[tris[:,0]],pos[tris[:,2]]-pos[tris[:,0]])
vn=np.zeros_like(pos)
for k in range(3): np.add.at(vn,tris[:,k],fn)
vn/=np.linalg.norm(vn,axis=1,keepdims=True)+1e-12
# UV islands (connected through shared vertex indices)
n=len(pos); e=np.concatenate([tris[:,[0,1]],tris[:,[1,2]],tris[:,[2,0]]])
_,isl=connected_components(coo_matrix((np.ones(len(e)),(e[:,0],e[:,1])),shape=(n,n)),directed=False)
P=np.full((S,S,3),np.nan,np.float32); N=np.zeros((S,S,3),np.float32); I=np.full((S,S),-1,np.int16)
px=uv*S-0.5
for t in tris:
    a,b,c=px[t]
    x0=int(max(0,np.floor(min(a[0],b[0],c[0])))); x1=int(min(S-1,np.ceil(max(a[0],b[0],c[0]))))
    y0=int(max(0,np.floor(min(a[1],b[1],c[1])))); y1=int(min(S-1,np.ceil(max(a[1],b[1],c[1]))))
    if x1<x0 or y1<y0: continue
    xs,ys=np.meshgrid(np.arange(x0,x1+1),np.arange(y0,y1+1))
    den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
    if abs(den)<1e-12: continue
    l0=((b[1]-c[1])*(xs-c[0])+(c[0]-b[0])*(ys-c[1]))/den
    l1=((c[1]-a[1])*(xs-c[0])+(a[0]-c[0])*(ys-c[1]))/den
    l2=1-l0-l1
    m=(l0>=-0.02)&(l1>=-0.02)&(l2>=-0.02)
    if not m.any(): continue
    L=np.stack([l0[m],l1[m],l2[m]],1)
    P[ys[m],xs[m]]=L@pos[t]; N[ys[m],xs[m]]=L@vn[t]; I[ys[m],xs[m]]=isl[t[0]]
np.savez_compressed('posmap.npz',P=P,N=N,I=I)
ids,counts=np.unique(I[I>=0],return_counts=True)
print('islands',list(zip(ids,counts)))
