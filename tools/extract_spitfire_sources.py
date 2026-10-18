#!/usr/bin/env python3
"""Fetch the two public NACA source reports and expose page-indexed source text.

Research preparation only; no simulator or aircraft-profile imports this file.
Run with pypdf installed in an isolated environment. Downloads/results are ignored.
"""
import json
import base64
import re
import sys
from pathlib import Path
from urllib.request import urlopen
from pypdf import PdfReader

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT/'results/spitfire-source'
REPORTS = {'L334': '19930092582', 'L353': '19930092581'}


def ocr_review(pdf, name):
    """Second reading of ambiguous configuration/table scans, never auto-adopted."""
    import pymupdf
    tessdata = ROOT/'build/reference-tessdata'
    tessdata.mkdir(parents=True, exist_ok=True)
    language = tessdata/'eng.traineddata'
    if not language.exists():
        # Official Tesseract English recognition data; data only, no installer.
        language.write_bytes(urlopen('https://raw.githubusercontent.com/tesseract-ocr/tessdata_fast/main/eng.traineddata', timeout=60).read())
    with pymupdf.open(pdf) as document:
        for number in ({'L334': (3, 4, 15), 'L353': (5, 9)}[name]):
            page = document[number-1]
            page.get_pixmap(dpi=200).save(DEST/f'{name}-page-{number}.png')
            tp = page.get_textpage_ocr(full=True, dpi=300, tessdata=str(tessdata))
            text = page.get_text(textpage=tp, sort=True)
            (DEST/f'{name}-page-{number}-ocr.txt').write_text(text)
            print(f'\n=== {name} SCAN OCR PDF PAGE {number} (not visual verification) ===\n{text}')
            if '--log-review-images' in sys.argv and (name, number) in (('L334', 4), ('L353', 9)):
                # Public source scans for clients whose log API transports text
                # but cannot download binary artifacts. No image is generated.
                png = page.get_pixmap(dpi=120, colorspace=pymupdf.csGRAY).tobytes('png')
                print(f'REVIEW_SCAN {name} {number} '+base64.b64encode(png).decode())


def main():
    DEST.mkdir(parents=True, exist_ok=True)
    index = {}
    for name, document in REPORTS.items():
        url = f'https://ntrs.nasa.gov/api/citations/{document}/downloads/{document}.pdf'
        data = urlopen(url, timeout=60).read()
        if not data.startswith(b'%PDF'):
            raise RuntimeError(f'{name}: source response is not a PDF')
        pdf = DEST/f'{name}.pdf'
        pdf.write_bytes(data)
        reader = PdfReader(pdf)
        pages = []
        for i, page in enumerate(reader.pages, start=1):
            text = page.extract_text(extraction_mode='layout') or ''
            pages.append({'pdf_page_1based': i, 'text': text})
        (DEST/f'{name}-pages.json').write_text(json.dumps(pages, indent=2)+'\n')
        text = '\n'.join(f'\n=== {name} PDF PAGE {p["pdf_page_1based"]} ===\n{p["text"]}' for p in pages)
        (DEST/f'{name}.txt').write_text(text)
        index[name] = {'source_url': url, 'pdf_pages': len(pages),
                       'text_characters': sum(len(p['text']) for p in pages)}
        # Printed source is a public US-government report. Expose substantive
        # text for review; page figures/tables still require visual verification.
        # PDF layout extraction has huge inter-character padding. Keep the raw
        # artifact, but make complete prose pages available in review logs.
        readable = '\n'.join(re.sub(r'[ \t]+', ' ', line).strip() for line in text.splitlines())
        print(readable[:160000])
        if index[name]['text_characters'] < 3000:
            raise RuntimeError(f'{name}: insufficient embedded text; OCR/visual reading is still required')
        if '--ocr-review' in sys.argv:
            ocr_review(pdf, name)
    (DEST/'index.json').write_text(json.dumps(index, indent=2)+'\n')
    print(json.dumps(index, indent=2))


if __name__ == '__main__':
    main()
