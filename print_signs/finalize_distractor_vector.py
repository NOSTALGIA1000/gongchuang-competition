"""Replace preview raster page with the downloaded sign's vector artwork."""
from pathlib import Path
import fitz

root = Path('F:/工创比赛/国标标识打印包')
target = root / '四张干扰项_A4无文字打印.pdf'
original = fitz.open(target)
vector = fitz.open(root / '08_干扰项4张/注意行人_统一黄色.pdf')
result = fitz.open()
page = result.new_page(width=original[0].rect.width, height=original[0].rect.height)
side = 180 * 72 / 25.4
frame = fitz.Rect((page.rect.width-side)/2, (page.rect.height-side)/2,
                  (page.rect.width+side)/2, (page.rect.height+side)/2)
page.show_pdf_page(frame, vector, 0, keep_proportion=True)
result.insert_pdf(original, from_page=1)
result.set_metadata(original.metadata)
temp = target.with_suffix('.tmp.pdf')
result.save(temp, garbage=4, deflate=True)
original.close()
result.close()
temp.replace(target)
check = fitz.open(target)
assert len(check) == 4
assert all(not p.get_text().strip() for p in check)
assert len(check[0].get_images()) == 0, 'First page must retain vector paths'
assert len(check[0].get_drawings()) > 0
check[0].get_pixmap(matrix=fitz.Matrix(1.2,1.2),alpha=False).save(str(root/'08_干扰项4张/高清打印检查.png'))
print('PASS: 4 pages, zero text, first page vector paths, no raster images')
