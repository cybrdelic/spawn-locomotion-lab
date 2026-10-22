"""One physical feedback actor; terrain and four-effector intent share its head."""
import torch
from torch import nn
from torch.nn import functional as F
from residual.actor import Actor as Residual
class Actor(Residual):
    observation_size=216
    def __init__(self,initialize_base=True):
        super().__init__(initialize_base=initialize_base)
        self.context=nn.Linear(65,128,bias=False);nn.init.zeros_(self.context.weight);self.contract_version.fill_(5)
    def normalized(self,normal):
        normal=normal.clamp(-10,10);raw=normal[...,:150]*self.base.std+self.base.mean
        hidden=F.relu(self.base.layers[0](normal[...,:150]));hidden=F.relu(self.base.layers[2](hidden))
        prior=self.base.layers[4](hidden).clamp(-1,1)
        state=torch.cat([hidden,raw[...,145:150],normal[...,150:151]],-1)
        correction=self.head[2](F.elu(self.head[0](state)+self.context(normal[...,151:])))
        return prior+self.correction_bound*torch.tanh(correction)
    def forward(self,observation):
        assert observation.shape[-1]==self.observation_size
        return self.normalized(torch.cat([((observation[...,:150]-self.base.mean)/self.base.std).clamp(-10,10),observation[...,150:]],-1))
    @classmethod
    def from_residual(cls,previous):
        actor=cls(initialize_base=False);state=previous.state_dict();state['context.weight']=actor.context.weight.detach().clone();state['contract_version']=torch.tensor(5)
        actor.load_state_dict(state);return actor
