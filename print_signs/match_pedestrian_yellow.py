"""Match the vector sign's print color to the three existing warning images."""
from pathlib import Path
import fitz

folder = Path('F:/工创比赛/国标标识打印包/08_干扰项4张')
source = folder / '注意行人_国标矢量.svg'
text = source.read_text(encoding='utf-8')
assert text.count('#FFEE33') == 1
variant = folder / '注意行人_统一黄色.svg'
variant.write_text(text.replace('#FFEE33', '#E0BC21'), encoding='utf-8')
svg = fitz.open(variant)
pdf = fitz.open('pdf', svg.convert_to_pdf())
pdf.save(variant.with_suffix('.pdf'))
pdf[0].get_pixmap(matrix=fitz.Matrix(4,4), alpha=False).save(str(variant.with_suffix('.png')))
print('Yellow matched: RGB(224,188,33), #E0BC21. Original SVG preserved.')
