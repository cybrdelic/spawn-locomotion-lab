"""Existing loopback transport, explicitly supplied one controller factory."""
from pathlib import Path
import os,sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'policy'))
os.environ['LOCOMOTION_CONTROLLER']='unified-contact'
from unified_runtime.controller import Controller
from policy.server import serve
if __name__=='__main__':serve(Controller)
