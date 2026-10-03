from pypdf import PdfReader
pdf = r"D:\microsoft下载\P020240710575371540000.pdf"
out = r"D:\Autotaxi\tmp\font-standard.txt"
p = PdfReader(pdf)
with open(out, "w", encoding="utf-8") as f:
    for i, page in enumerate(p.pages):
        f.write(f"---PAGE {i+1}---\n")
        f.write(page.extract_text() or "")
        f.write("\n")
