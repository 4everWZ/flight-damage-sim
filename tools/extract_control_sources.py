#!/usr/bin/env python3
"""Retrieve fixed official control-surface reports; never adopt OCR as data.

Uses the existing isolated reference environment. No simulator imports,
arbitrary download URLs, installers, credentials or production model changes.
"""
import argparse
import base64
import json
from pathlib import Path
from urllib.request import urlopen

import pymupdf

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT/'results/control-source'
REPORTS = {
    'NACA0009-double': ('19930092885', (4, 5, 7, 19, 20, 37, 38, 39, 40)),
    'NACA0015-plain': ('19930092872', (4, 5, 6, 15, 16, 17, 18)),
    # L-290 references 7 and 4: coefficient/axis definitions and facility.
    'NACA721-conventions': ('19930091799', (9, 21, 27)),
    'NACA387-facility': ('19930091086', (5,)),
    'CR3439-ms0313': ('19830008016', (3, 5, 6, 7, 8, 9, 18, 31, 32, 33)),
}
LOG_PAGES = ({('NACA0009-double', page) for page in (5, 7, 20, 37)}
             | {('NACA0015-plain', 17), ('NACA721-conventions', 27)}
             | {('CR3439-ms0313', page) for page in (18, 31, 32, 33)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reports', nargs='+', choices=REPORTS, default=list(REPORTS))
    parser.add_argument('--log-review-images', action='store_true')
    args = parser.parse_args()
    DEST.mkdir(parents=True, exist_ok=True)
    index = {}
    for name in args.reports:
        identifier, selected = REPORTS[name]
        url = f'https://ntrs.nasa.gov/api/citations/{identifier}/downloads/{identifier}.pdf'
        data = urlopen(url, timeout=60).read()
        if not data.startswith(b'%PDF'):
            raise RuntimeError(f'{name}: response is not a PDF')
        pdf = DEST/f'{name}.pdf'
        pdf.write_bytes(data)
        with pymupdf.open(pdf) as document:
            pages = [{'pdf_page_1based': i+1, 'text': page.get_text(sort=True)}
                     for i, page in enumerate(document)]
            (DEST/f'{name}-pages.json').write_text(json.dumps(pages, indent=2)+'\n')
            if name == 'NACA387-facility':
                for record in pages:
                    print(f'CONTROL_SOURCE_TEXT {name} PDF{record["pdf_page_1based"]}\n{record["text"]}')
            for number in selected:
                page = document[number-1]
                if ((name == 'NACA0009-double' and number in (4, 5, 7))
                        or name == 'NACA721-conventions'
                        or (name == 'CR3439-ms0313' and number <= 9)):
                    print(f'CONTROL_SOURCE_TEXT {name} PDF{number}\n{pages[number-1]["text"]}')
                dpi = 300 if name == 'CR3439-ms0313' else 200
                png = page.get_pixmap(dpi=dpi, colorspace=pymupdf.csGRAY).tobytes('png')
                (DEST/f'{name}-page-{number}.png').write_bytes(png)
                if args.log_review_images and (name, number) in LOG_PAGES:
                    # Bounded text transport for clients unable to read binary
                    # artifacts. The bytes are original source scan renders.
                    encoded = base64.b64encode(png).decode()
                    chunks = [encoded[i:i+32000] for i in range(0, len(encoded), 32000)]
                    for i, chunk in enumerate(chunks):
                        print(f'CONTROL_SCAN {name} {number} {i+1}/{len(chunks)} {chunk}')
            index[name] = {'source_url': url, 'pdf_pages': len(document),
                           'scan_dpi': 300 if name == 'CR3439-ms0313' else 200,
                           'selected_pdf_pages_1based': list(selected),
                           'meaning': 'Original scan/text only; no coefficient extraction or physical validation claimed.'}
    (DEST/'index.json').write_text(json.dumps(index, indent=2)+'\n')
    print(json.dumps(index, indent=2))


if __name__ == '__main__':
    main()
