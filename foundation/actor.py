"""Standalone preserved feedback backbone. Loading an artifact needs no trainer."""
from pathlib import Path
import torch
from torch import nn
from foundation.coordinates import ROOT,SIZE
SOURCE=ROOT/'policy/adaptation-06/int_models/model_0000000120.pt'
class Actor(nn.Module):
    observation_size=150
    def __init__(self,initialize=True):
        super().__init__();self.layers=nn.Sequential(nn.Linear(SIZE,1024),nn.ReLU(),nn.Linear(1024,512),nn.ReLU(),nn.Linear(512,28))
        self.register_buffer('mean',torch.zeros(SIZE));self.register_buffer('std',torch.ones(SIZE));self.register_buffer('action_mean',torch.zeros(28));self.register_buffer('action_std',torch.ones(28))
        if initialize:
            w=torch.load(SOURCE,map_location='cpu',weights_only=True)
            with torch.no_grad():
                self.layers[0].weight.zero_();self.layers[0].weight[:,:145].copy_(w['_model._actor_layers.0.weight']);self.layers[0].bias.copy_(w['_model._actor_layers.0.bias'])
                self.layers[2].weight.copy_(w['_model._actor_layers.2.weight']);self.layers[2].bias.copy_(w['_model._actor_layers.2.bias'])
                self.layers[4].weight.copy_(w['_model._action_dist._mean_net.weight']);self.layers[4].bias.copy_(w['_model._action_dist._mean_net.bias'])
                self.mean[:145].copy_(w['_obs_norm._mean']);self.std[:145].copy_(w['_obs_norm._std']);self.action_mean.copy_(w['_a_norm._mean']);self.action_std.copy_(w['_a_norm._std'])
    def forward(self,observation):return self.layers(((observation-self.mean)/self.std).clamp(-10,10))
    def target(self,observation):return self.action_mean+self(observation).clamp(-1,1)*self.action_std
    @classmethod
    def load(cls,path):
        state=torch.load(path,map_location='cpu',weights_only=True)
        if 'context.weight' in state:
            from natural.actor import Actor as ContactActor
            actor=ContactActor(initialize_base=False)
        elif 'head.0.weight' in state and 'base.layers.0.weight' in state:
            from residual.actor import Actor as ResidualActor
            actor=ResidualActor(initialize_base=False)
            state.setdefault('contract_version',torch.tensor(1))
        else:actor=cls(initialize=False)
        actor.load_state_dict(state);actor.eval();return actor
