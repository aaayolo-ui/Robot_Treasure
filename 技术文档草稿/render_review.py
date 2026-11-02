from pathlib import Path
import importlib.util,json,sys
from PIL import Image,ImageDraw,ImageFont
root=Path(__file__).resolve().parent
qa=root/'排版检查'
skill=Path('C:/Users/Administrator/.codex/plugins/cache/openai-primary-runtime/documents/26.1029.11837/skills/documents/render_docx.py')
spec=importlib.util.spec_from_file_location('docx_renderer',skill)
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
# Windows bundle has no LibreOffice. Use the installed Word's PDF export as
# the conversion backend, retaining packaged render_docx.py page rasterization.
mod.convert_to_pdf=lambda *args,**kwargs:(str(qa/'preview.pdf'),'Native Word PDF export')
docx=next(root.glob('*.docx'))
pages=mod.rasterize(str(docx),str(qa/'pages'),110,False,False)
font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',22)
contacts=[]
for start in range(0,len(pages),6):
    sheet=Image.new('RGB',(1530,1360),'#DFE5E9');draw=ImageDraw.Draw(sheet)
    for j,path in enumerate(pages[start:start+6]):
        im=Image.open(path).convert('RGB');im.thumbnail((495,635))
        x=(j%3)*510;y=(j//3)*680
        # Preserve readable page-size preview; unused bottom space is reserved
        # for page labels rather than squeezing the original aspect ratio.
        im.thumbnail((495,975));sheet.paste(im,(x+(495-im.width)//2,y+32))
        draw.text((x+12,y+4),'第 '+str(start+j+1)+' 页',font=font,fill='#153C59')
    out=qa/('contact-'+str(start//6+1)+'.jpg');sheet.save(out,quality=90);contacts.append(str(out))
(qa/'render_record.json').write_text(json.dumps(dict(renderer='packaged render_docx.py with native Word PDF backend',pages=len(pages),contacts=contacts),ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(dict(pages=len(pages),contacts=contacts),ensure_ascii=False))
