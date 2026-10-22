"""One all-state physical feedback field over a frozen locomotion prior.

Low motion is not evidence of safe balance. New controllers retain correction
authority at rest; old artifact versions preserve their historical semantics.
"""
from pathlib import Path
import json
import torch
from torch import nn
from torch.nn import functional as F
from foundation.actor import Actor as Backbone
class Actor(nn.Module):
    observation_size=151
    def __init__(self,calibration=None,initialize_base=True):
        super().__init__();self.base=Backbone(initialize=initialize_base);self.base.requires_grad_(False)
        self.head=nn.Sequential(nn.Linear(518,128),nn.ELU(),nn.Linear(128,28))
        nn.init.zeros_(self.head[2].weight);nn.init.zeros_(self.head[2].bias)
        c=calibration or (json.loads((Path(__file__).parent/'calibration.json').read_text(encoding='utf-8')) if initialize_base else {'quietSpeedLimit':.1,'responseSpeed':.3,'heightToSpeed':.65})
        self.register_buffer('quiet_energy',torch.tensor(c['quietSpeedLimit']**2))
        self.register_buffer('response_energy',torch.tensor(c['responseSpeed']**2))
        self.register_buffer('height_to_speed',torch.tensor(c['heightToSpeed']))
        self.register_buffer('contract_version',torch.tensor(3))
        self.register_buffer('correction_bound',torch.tensor(2.))
    def normalized(self,normal):
        normal=normal.clamp(-10,10);raw=normal[...,:150]*self.base.std+self.base.mean
        h=F.relu(self.base.layers[0](normal[...,:150]));h=F.relu(self.base.layers[2](h));baseline=self.base.layers[4](h)
        # Full normalized target authority; preserve old self-contained artifacts.
        if self.contract_version>=2:baseline=baseline.clamp(-1,1)
        correction=self.correction_bound*torch.tanh(self.head(torch.cat([h,raw[...,145:150],normal[...,150:151]],dim=-1)))
        if self.contract_version>=3:return baseline+correction
        demand=(normal[...,150]-self.quiet_energy).clamp_min(0)+raw[...,142].square()+(raw[...,149]*self.height_to_speed).square()
        t=(demand/self.response_energy).clamp(0,1);gate=t*t*(3-2*t)
        return baseline+gate.unsqueeze(-1)*correction
    def forward(self,observation):return self.normalized(torch.cat([((observation[...,:150]-self.base.mean)/self.base.std).clamp(-10,10),observation[...,150:151]],dim=-1))
    def target(self,observation):return self.base.action_mean+self(observation).clamp(-1,1)*self.base.action_std
