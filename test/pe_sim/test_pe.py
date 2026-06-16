#!/usr/bin/env python3
# PE unit tests
import random, cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

MAX_N=512; C_ROW_STRIDE=MAX_N

def gen(rows, cols, sp, seed=42, is_B=False):
    rng=random.Random(seed)
    npr=max(1,int(cols*sp))
    rd,ca,va=[],[],[]; off=0
    for r in range(rows):
        cs=set()
        while len(cs)<npr: c=rng.randint(0,cols-1); cs.add(c) if c not in cs else None
        for c in sorted(cs):
            v=(r*37+c*13+1)%7+1; ca.append(c); va.append(v)
        if is_B: d=(off<<32)|(0<<16)|npr
        else: d=(off<<32)|(npr<<16)|r
        rd.append(d); off+=npr
    return rd,ca,va,off

def gold(Ad,Ac,Av,Bd,Bc,Bv,M,N):
    C=[[0]*MAX_N for _ in range(MAX_N)]
    for ri in range(M):
        gid=Ad[ri]&0xFFFF; nnza=(Ad[ri]>>16)&0xFFFF; st=(Ad[ri]>>32)&0xFFFFFFFF
        for t in range(nnza):
            k=Ac[st+t]&0xFFFF; a=Av[st+t]
            bn=Bd[k]&0xFFFF; bs=(Bd[k]>>32)&0xFFFFFFFF
            for u in range(bn): j=Bc[bs+u]&0xFFFF; b=Bv[bs+u]; C[gid][j]+=a*b
    gv={}; gf=[[0.0]*N for _ in range(M)]
    for ri in range(M):
        gid=Ad[ri]&0xFFFF
        for j in range(N):
            a=gid*C_ROW_STRIDE+j; v=C[gid][j]
            if v!=0: gv[a]=v
            gf[gid][j]=float(v)
    return gv,gf

async def LAd(dut,Ad,Ac,Av):
    for i,d in enumerate(Ad): dut.a_desc_we.value=1;dut.a_desc_waddr.value=i;dut.a_desc_wdata.value=d;await RisingEdge(dut.aclk)
    dut.a_desc_we.value=0
    for i,v in enumerate(Ac): dut.a_col_we.value=1;dut.a_col_waddr.value=i;dut.a_col_wdata.value=v;await RisingEdge(dut.aclk)
    dut.a_col_we.value=0
    for i,v in enumerate(Av): dut.a_val_we.value=1;dut.a_val_waddr.value=i;dut.a_val_wdata.value=v;await RisingEdge(dut.aclk)
    dut.a_val_we.value=0

async def LBd(dut,Bd,Bc,Bv):
    for i,d in enumerate(Bd): dut.b_desc_we.value=1;dut.b_desc_waddr.value=i;dut.b_desc_wdata.value=d;await RisingEdge(dut.aclk)
    dut.b_desc_we.value=0
    for i,v in enumerate(Bc): dut.b_col_we.value=1;dut.b_col_waddr.value=i;dut.b_col_wdata.value=v;await RisingEdge(dut.aclk)
    dut.b_col_we.value=0
    for i,v in enumerate(Bv): dut.b_val_we.value=1;dut.b_val_waddr.value=i;dut.b_val_wdata.value=v;await RisingEdge(dut.aclk)
    dut.b_val_we.value=0

async def rst(dut):
    dut.aresetn.value=0; cocotb.start_soon(Clock(dut.aclk,10,units='ns').start())
    await ClockCycles(dut.aclk,10); dut.aresetn.value=1; await ClockCycles(dut.aclk,5)
    dut.start.value=0;dut.row_count.value=0;dut.cbuf_wr_ready.value=0
    dut.a_desc_we.value=0;dut.a_col_we.value=0;dut.a_val_we.value=0
    dut.b_desc_we.value=0;dut.b_col_we.value=0;dut.b_val_we.value=0

async def run(dut,rc,to=500000):
    dut.row_count.value=rc;dut.cbuf_wr_ready.value=1;dut.start.value=1;await RisingEdge(dut.aclk);dut.start.value=0
    cp={};dc=0
    for cy in range(to):
        await RisingEdge(dut.aclk)
        if dut.cbuf_wr_valid.value and dut.cbuf_wr_ready.value: cp[int(dut.cbuf_wr_addr.value)]=int(dut.cbuf_wr_data.value)
        if int(dut.done.value): dc=cy;break
    else: assert False,f"timeout {to}"
    await ClockCycles(dut.aclk,50)
    for _ in range(100):
        await RisingEdge(dut.aclk)
        if dut.cbuf_wr_valid.value and dut.cbuf_wr_ready.value: cp[int(dut.cbuf_wr_addr.value)]=int(dut.cbuf_wr_data.value)
    return cp,dc

def vfy(dut,M,N,Ad,gf,cp):
    e=0
    for ri in range(M):
        gid=Ad[ri]&0xFFFF;b=gid*C_ROW_STRIDE
        for j in range(N):
            exp=int(gf[gid][j]);act=cp.get(b+j,0)
            if act!=exp:
                if e<5:dut._log.error("C[%d][%d]: got %d, exp %d",gid,j,act,exp)
                e+=1
    return e

@cocotb.test()
async def test_pe_3x20(dut):
    """3x20 with 6 nnz/row"""
    M,K,N=3,20,20;SP=0.30
    Ad,Ac,Av,An=gen(M,K,SP,42,False); Bd,Bc,Bv,Bn=gen(K,N,SP,77,True)
    gv,gf=gold(Ad,Ac,Av,Bd,Bc,Bv,M,N)
    dut._log.info("3x20: A_nnz=%d B_nnz=%d golden_nnz=%d",An,Bn,len(gv))
    await rst(dut); await LAd(dut,Ad,Ac,Av); await LBd(dut,Bd,Bc,Bv)
    cp,cy=await run(dut,M,to=500000)
    dut._log.info("Done cycle=%d",cy); e=vfy(dut,M,N,Ad,gf,cp); assert e==0,f"{e} mismatches"
    dut._log.info("3x20 PASSED")

@cocotb.test()
async def test_pe_4x20(dut):
    """4x20 with 6 nnz/row"""
    M,K,N=4,20,20;SP=0.30
    Ad,Ac,Av,An=gen(M,K,SP,42,False); Bd,Bc,Bv,Bn=gen(K,N,SP,77,True)
    gv,gf=gold(Ad,Ac,Av,Bd,Bc,Bv,M,N)
    dut._log.info("4x20: A_nnz=%d B_nnz=%d golden_nnz=%d",An,Bn,len(gv))
    await rst(dut); await LAd(dut,Ad,Ac,Av); await LBd(dut,Bd,Bc,Bv)
    cp,cy=await run(dut,M,to=1000000)
    dut._log.info("Done cycle=%d",cy); e=vfy(dut,M,N,Ad,gf,cp); assert e==0,f"{e} mismatches"
    dut._log.info("4x20 PASSED")

@cocotb.test()
async def test_pe_20x20(dut):
    """20x20, 30% sparsity"""
    M,K,N=20,20,20;SP=0.30
    Ad,Ac,Av,An=gen(M,K,SP,42,False); Bd,Bc,Bv,Bn=gen(K,N,SP,77,True)
    gv,gf=gold(Ad,Ac,Av,Bd,Bc,Bv,M,N)
    dut._log.info("20x20: A_nnz=%d B_nnz=%d golden_nnz=%d",An,Bn,len(gv))
    await rst(dut); await LAd(dut,Ad,Ac,Av); await LBd(dut,Bd,Bc,Bv)
    cp,cy=await run(dut,M,to=5000000)
    dut._log.info("Done cycle=%d",cy); e=vfy(dut,M,N,Ad,gf,cp); assert e==0,f"{e} mismatches"
    dut._log.info("20x20 PASSED")

@cocotb.test()
async def test_pe_50x50(dut):
    """50x50, 30% sparsity"""
    M,K,N=50,50,50;SP=0.30
    Ad,Ac,Av,An=gen(M,K,SP,42,False); Bd,Bc,Bv,Bn=gen(K,N,SP,77,True)
    gv,gf=gold(Ad,Ac,Av,Bd,Bc,Bv,M,N)
    dut._log.info("50x50: A_nnz=%d B_nnz=%d golden_nnz=%d",An,Bn,len(gv))
    await rst(dut); await LAd(dut,Ad,Ac,Av); await LBd(dut,Bd,Bc,Bv)
    cp,cy=await run(dut,M,to=10000000)
    dut._log.info("Done cycle=%d",cy); e=vfy(dut,M,N,Ad,gf,cp); assert e==0,f"{e} mismatches"
    dut._log.info("50x50 PASSED")
