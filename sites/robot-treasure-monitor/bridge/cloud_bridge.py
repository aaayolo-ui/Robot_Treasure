"""Relay existing local monitor snapshots and bounded serial commands; no motor writes."""
import argparse,base64,ctypes,json,os,sys,time,uuid,urllib.request,urllib.error
from pathlib import Path

SITE='https://robot-treasure-monitor-1026.nicemug257.chatgpt.site'
CONFIG=Path(os.environ.get('LOCALAPPDATA',str(Path.home())))/'RobotTreasureMonitor'/'site.json'
class Blob(ctypes.Structure):
    _fields_=[('size',ctypes.c_ulong),('data',ctypes.POINTER(ctypes.c_ubyte))]
def dpapi(raw,protect):
    buf=ctypes.create_string_buffer(raw);source=Blob(len(raw),ctypes.cast(buf,ctypes.POINTER(ctypes.c_ubyte)));dest=Blob()
    if protect:
        ok=ctypes.windll.crypt32.CryptProtectData(ctypes.byref(source),'RobotTreasure monitor',None,None,None,1,ctypes.byref(dest))
    else:
        ok=ctypes.windll.crypt32.CryptUnprotectData(ctypes.byref(source),None,None,None,None,1,ctypes.byref(dest))
    if not ok:raise ctypes.WinError()
    try:return ctypes.string_at(dest.data,dest.size)
    finally:ctypes.windll.kernel32.LocalFree(dest.data)
class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self,*args,**kwargs):return None
opener=urllib.request.build_opener(NoRedirect)
def call(url,payload=None,token=None):
    headers={'Accept':'application/json'}
    if token:
        if not url.startswith(SITE+'/'):raise ValueError('Credential destination mismatch')
        headers['OAI-Sites-Authorization']='Bearer '+token
    data=None if payload is None else json.dumps(payload,ensure_ascii=False).encode('utf-8')
    if data is not None:headers['Content-Type']='application/json'
    with opener.open(urllib.request.Request(url,data=data,headers=headers),timeout=15) as r:return json.load(r)
def execute(command,local,session,seen,now_ms=None):
    now_ms=int(time.time()*1000) if now_ms is None else now_ms
    if not command or command.get('id')==seen or command.get('session')!=session:return None
    if not 0<=now_ms-command.get('issuedAt',0)<15000:return None
    action=command.get('action')
    if action not in ('connect','disconnect','select','reset-position'):return None
    endpoint='/api/reset-position' if action=='reset-position' else '/api/serial'
    body={} if action=='reset-position' else dict(action=action,port=command.get('port',''))
    try:
        call(local+endpoint,body)
        return dict(id=command['id'],result={'ok':True})
    except (OSError,ValueError):return dict(id=command['id'],result={'ok':False,'error':'电脑串口操作失败，请检查串口占用或连接'})
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--configure',action='store_true');parser.add_argument('--once',action='store_true');parser.add_argument('--local',default='http://127.0.0.1:8766')
    args=parser.parse_args()
    if args.configure:
        if sys.stdin.isatty():
            import getpass
            line=getpass.getpass('Bridge setup JSON (hidden): ')
        else:line=sys.stdin.readline()
        data=json.loads(line)
        if data['site']!=SITE or not data.get('token'):raise ValueError('Invalid Site credential')
        CONFIG.parent.mkdir(parents=True,exist_ok=True)
        CONFIG.write_text(json.dumps(dict(site=SITE,credential=base64.b64encode(dpapi(data['token'].encode(),True)).decode())),encoding='utf-8')
        print('Bridge credential saved with Windows user encryption; no serial connection opened.',flush=True);return
    data=json.loads(CONFIG.read_text(encoding='utf-8'))
    if data['site']!=SITE:raise ValueError('Unexpected Site URL')
    token=dpapi(base64.b64decode(data['credential']),False).decode()
    lock=CONFIG.with_suffix('.lock').open('a+b')
    import msvcrt
    try:lock.seek(0);lock.write(b'0');lock.flush();lock.seek(0);msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
    except OSError:print('Cloud bridge already running.');return
    session=str(uuid.uuid4());ack=None;seen=None;retry=1
    while True:
        try:
            snap=call(args.local+'/api/laser');snap.pop('raw_log_path',None)
            ports=call(args.local+'/api/ports')['ports'];path=call(args.local+'/api/path?after=0');path['points']=path['points'][-800:]
            result=call(SITE+'/api/bridge',dict(snapshot=snap,ports=ports,path=path,session=session,ack=ack),token)
            new_ack=execute(result.get('command'),args.local,session,seen)
            if new_ack:ack=new_ack;seen=ack['id']
            if args.once:print('Uploaded actual local snapshot; serial status: '+str(snap['connected']),flush=True);return
            retry=1;time.sleep(1)
        except KeyboardInterrupt:return
        except (OSError,ValueError) as e:
            print('Bridge temporarily unavailable: '+type(e).__name__,flush=True)
            if args.once:raise SystemExit(1)
            time.sleep(retry);retry=min(15,retry*2)
if __name__=='__main__':main()
