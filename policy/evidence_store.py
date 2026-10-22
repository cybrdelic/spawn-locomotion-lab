"""Immutable tagged native traces; ordinary UI pauses may update latest only."""
from pathlib import Path
import json,re
class EvidenceConflict(Exception):pass
class EvidenceStore:
    def __init__(self,folder):self.folder=Path(folder);self.last=None
    def reset(self):self.last=None
    def save(self,payload,tag=None):
        if tag is not None and (not isinstance(tag,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,80}',tag)):
            raise EvidenceConflict('Invalid evidence tag')
        frames=payload['frames'];key=(tag,payload['checkpoint'],len(frames),frames[-1]['time'] if frames else 0)
        if tag and key==self.last:return 'native-'+tag+'.json'
        value={**payload,'captureTag':tag};text=json.dumps(value,separators=(',',':'))
        (self.folder/'policy-native-latest.json').write_text(text,encoding='utf-8')
        if not tag:return None
        path=self.folder/('native-'+tag+'.json')
        try:
            with path.open('x',encoding='utf-8') as f:f.write(text)
        except FileExistsError:raise EvidenceConflict('Evidence tag already belongs to another capture')
        self.last=key;return path.name
