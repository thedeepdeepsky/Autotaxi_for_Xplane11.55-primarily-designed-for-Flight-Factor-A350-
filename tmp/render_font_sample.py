from PIL import Image, ImageDraw, ImageFont
from pathlib import Path
font_path=Path(r'D:\Autotaxi\assets\OpenTaxiwayMandatorySign.ttf')
out=Path(r'D:\Autotaxi\build-release\sign-preview\open-runway-font-sample.png')
img=Image.new('RGB',(1200,360),(23,25,28)); d=ImageDraw.Draw(img)
font=ImageFont.truetype(str(font_path),64)
label=ImageFont.truetype(r'C:\Windows\Fonts\consola.ttf',22)
d.text((45,32),'MH/T 6011-2015 / Open Taxiway Mandatory Sign',font=label,fill=(225,232,235))
box=(70,105,1130,245); d.rectangle(box,fill=(241,194,48),outline=(16,18,20),width=4)
x=125; y=140
for segment in ('D','36L','18R'):
    d.text((x,y),segment,font=font,fill=(10,12,14))
    x += d.textlength(segment,font=font) + 35
    if segment != '18R':
        d.rectangle((int(x-18),137,int(x-14),215),fill=(10,12,14))
        x += 35
d.text((70,285),'Vertical separators are drawn as | and included in measured board width.',font=label,fill=(174,183,189))
img.save(out)
