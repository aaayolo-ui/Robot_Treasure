"""List every new 16L source/artifact and its delta from the preserved 16K copy."""
from pathlib import Path
import difflib,hashlib,json,subprocess

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[3]
FW=REPO/'STM32/Keil_Project/Thief16L_TerminalEscape_FourLaser_Test'
OUTPUT=HERE/'FILES_16L.json'

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    paths=sorted([*FW.rglob('*'),*HERE.rglob('*'),REPO/'Docs/23_条件藏点与停车后脱身16L.md'])
    paths=[p for p in paths if p.is_file() and p!=OUTPUT]
    names=[str(p.relative_to(REPO)).replace('\\','/') for p in paths]
    # NUL byte transport avoids Windows text-mode CRLF and Git path quoting.
    ignored=subprocess.run(['git','check-ignore','--stdin','-z'],input=('\0'.join(names)+'\0').encode('utf-8'),
                           cwd=REPO,capture_output=True).stdout.decode('utf-8').split('\0')
    allowed=[(p,n) for p,n in zip(paths,names) if n not in set(ignored)]
    ref=json.loads((FW/'copy_reference.json').read_text(encoding='utf-8'))
    rows=ref['copied_files']
    # The source record schema is kept in the original copy manifest.
    source_project=Path(ref['source_project'])
    mapping={r['destination']:source_project/r['source'] for r in rows}
    files=[];added_lines=removed_lines=changed_copies=unchanged_copies=0
    for path,name in allowed:
        relative=str(path.relative_to(FW)).replace('\\','/') if path.is_relative_to(FW) else None
        source_path=mapping.get(relative)
        item={'path':name,'bytes':path.stat().st_size,'sha256':sha(path),'git_change':'added'}
        if source_path:
            old=source_path
            same=path.read_bytes()==old.read_bytes()
            item.update(copied_from=str(old.relative_to(REPO)).replace('\\','/'),copy_identical=same)
            unchanged_copies+=int(same);changed_copies+=int(not same)
            if not same:
                try:
                    a=old.read_text(encoding='utf-8').splitlines();b=path.read_text(encoding='utf-8').splitlines()
                    plus=minus=0
                    for op,i,j,k,l in difflib.SequenceMatcher(a=a,b=b,autojunk=False).get_opcodes():
                        if op!='equal':minus+=j-i;plus+=l-k
                    item.update(lines_added_vs_copy=plus,lines_removed_vs_copy=minus)
                    added_lines+=plus;removed_lines+=minus
                except UnicodeError:item['diff_kind']='binary_or_legacy_encoding'
        files.append(item)
    files.append({'path':str(OUTPUT.relative_to(REPO)).replace('\\','/'),'git_change':'added',
                  'sha256':None,'note':'This manifest excludes its own recursive hash.'})
    result={'scope':'16L-only additions; existing user changes and 16K history excluded',
            'added_file_count':len(files),'existing_files_modified':[],
            'copy_files_unchanged':unchanged_copies,'copy_files_changed':changed_copies,
            'changed_copy_line_summary':{'added':added_lines,'removed':removed_lines},
            'new_project':str(FW.relative_to(REPO)).replace('\\','/'),
            'preserved_low_level_file_count':117,'git_add_commit_push_executed':False,
            'hardware_run':False,'files':files}
    OUTPUT.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='files'},ensure_ascii=False))

if __name__=='__main__':main()
