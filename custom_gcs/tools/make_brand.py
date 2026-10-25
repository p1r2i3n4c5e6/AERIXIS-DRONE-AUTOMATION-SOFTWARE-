"""Generate the AERIXIS SVG logo set (traced from the raster logo in web/ui/brand, 2000x667 frame;
the wordmark has an I added after R, as the user asked, so it reads AERIXIS).

Usage: python3 tools/make_brand.py [output_dir]   (default: web/ui/brand)"""
import sys
from pathlib import Path
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent.parent / "web/ui/brand"
NAVY, TEAL = '#0B2347', '#04A9B3'
LIGHT, TEAL_D = '#F2F6FC', '#1BC0C8'

# Mark: the stylised "A" (two navy pieces) and the teal swoosh crossing it.
A_LEFT = 'M118 515 L325 172 L396 172 L407 153 L474 270 L412 296 L372 226 L178 515 Z'
A_RIGHT = 'M438 390 L505 342 L580 468 L485 468 Z'
SWOOSH = 'M200 515 L298 383 C380 322 480 278 598 253 C470 330 330 430 218 515 Z'
# Wordmark: the big mark is the A, then ERIXIS (geometric, wide; drawn as shapes so it never depends on a font).
# The traced raster had a second, small A (Λ) here; the user asked to remove it, so the letters move left by 262.
WORD = [
    'M886 278 H1060 V310 H926 V355 H1044 V389 H926 V436 H1060 V468 H886 Z',                    # E
    'M1107 468 V278 H1235 A61 61 0 0 1 1236 400 L1297 468 H1245 L1185 400 H1147 V468 Z '
    'M1147 310 V368 H1233 A29 29 0 0 0 1233 310 Z',                                            # R
    'M1333 278 H1380 V468 H1333 Z',                                                            # I (AERIXIS)
    'M1333 278 H1388 L1449 349 L1510 278 H1565 L1477 381 L1555 468 H1500 L1449 410 L1393 468 H1337 L1421 381 Z',  # X
    'M1620 278 H1667 V468 H1620 Z',                                                            # I
]
S_PATH = ('M1896 332 C1896 307 1882 295 1855 295 H1781 C1752 295 1739 309 1739 331 C1739 352 1752 362 1780 367 '
          'L1856 381 C1884 386 1896 398 1896 419 C1896 441 1882 451 1855 451 H1781 C1752 451 1739 440 1739 415')

def mark(a, s, cls=False):
    c = lambda n: f' class="{n}"' if cls else ''
    return (f'<g{c("aerixis-a")} fill="{a}"><path d="{A_LEFT}"/><path d="{A_RIGHT}"/></g>'
            f'<path{c("aerixis-swoosh")} fill="{s}" d="{SWOOSH}"/>')

def word(fill, cls=False):
    c = ' class="aerixis-word"' if cls else ''
    l = (lambda i: f' style="--i:{i}"') if cls else (lambda i: '')
    # Shifts sit on groups, not on the paths: the splash animates the paths' own transform.
    # E, R, I at their place (moved left 262); X, I, S a further 83 right to make room for the added I.
    return (f'<g{c} fill="{fill}" fill-rule="evenodd"><g transform="translate(-262 0)">' +
            ''.join(f'<path{l(i)} d="{d}"/>' for i, d in enumerate(WORD[:3])) +
            '<g transform="translate(83 0)">' + ''.join(f'<path{l(i)} d="{d}"/>' for i, d in enumerate(WORD) if i >= 3) +
            f'<path{l(len(WORD))} d="{S_PATH}" fill="none" stroke="{fill}" stroke-width="34"/></g></g></g>')

def svg(vb, body, title='AERIXIS'):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{vb}" role="img" aria-label="{title}">'
            f'<title>{title}</title>{body}</svg>\n')

FULL = '100 140 1640 390'
(OUT / 'aerixis-logo.svg').write_text(svg(FULL, mark(NAVY, TEAL) + word(NAVY)))
(OUT / 'aerixis-logo-dark.svg').write_text(svg(FULL, mark(LIGHT, TEAL_D) + word(LIGHT)))
MARK = '100 140 510 390'
(OUT / 'aerixis-mark.svg').write_text(svg(MARK, mark(NAVY, TEAL)))
(OUT / 'aerixis-mark-dark.svg').write_text(svg(MARK, mark(LIGHT, TEAL_D)))
# App icon / favicon: navy rounded tile, light mark, teal swoosh.
icon = (f'<rect x="0" y="0" width="512" height="512" rx="112" fill="{NAVY}"/>'
        f'<g transform="translate(256 262) scale(.74) translate(-354 -334)">{mark(LIGHT, TEAL_D)}</g>')
(OUT / 'aerixis-icon.svg').write_text(svg('0 0 512 512', icon))
# Same logo with class names (currentColor + var(--swoosh)) so CSS can animate it: pasted into the splash in index.html.
(OUT / 'aerixis-logo-anim.svg').write_text(svg(FULL, mark('currentColor', 'var(--swoosh)', True) + word('currentColor', True)))
