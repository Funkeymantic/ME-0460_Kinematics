#!/usr/bin/env python3
"""
ME 464 Kinematics Formula Sheet — PDF Generator
Uses DejaVu fonts for full Unicode math symbol support.
"""

from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import inch
from reportlab.lib import colors
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Table,
                                 TableStyle, HRFlowable, KeepTogether, PageBreak)
from reportlab.lib.enums import TA_LEFT, TA_CENTER, TA_RIGHT
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

# ── Register DejaVu fonts (full Unicode coverage) ────────────────────────────
FONT_DIR = '/usr/share/fonts/truetype/dejavu/'
pdfmetrics.registerFont(TTFont('DV',   FONT_DIR + 'DejaVuSans.ttf'))
pdfmetrics.registerFont(TTFont('DV-B', FONT_DIR + 'DejaVuSans-Bold.ttf'))
pdfmetrics.registerFont(TTFont('DVM',  FONT_DIR + 'DejaVuSansMono.ttf'))
pdfmetrics.registerFont(TTFont('DVM-B',FONT_DIR + 'DejaVuSansMono-Bold.ttf'))
pdfmetrics.registerFontFamily('DV',  normal='DV',  bold='DV-B')
pdfmetrics.registerFontFamily('DVM', normal='DVM', bold='DVM-B')

# ── Output ────────────────────────────────────────────────────────────────────
OUT = '/home/user/ME-0460_Kinematics/Notes/ME464_Formula_Sheet.pdf'

# ── Page setup ────────────────────────────────────────────────────────────────
doc = SimpleDocTemplate(OUT, pagesize=letter,
    rightMargin=0.6*inch, leftMargin=0.6*inch,
    topMargin=0.6*inch,  bottomMargin=0.6*inch,
    title='ME 464 Kinematics Formula Sheet',
    author='ME 464 — Chapter 2')

PW = letter[0] - 1.2*inch   # usable page width
PH = letter[1] - 1.2*inch

# ── Colors ────────────────────────────────────────────────────────────────────
NAVY   = colors.HexColor('#0c1a4a')
DBLUE  = colors.HexColor('#1a3a8f')
MBLUE  = colors.HexColor('#2255bb')
LBLUE  = colors.HexColor('#dde5f7')
LGRAY  = colors.HexColor('#f3f4f8')
MGRAY  = colors.HexColor('#c8cce0')
DGRAY  = colors.HexColor('#44475a')
WHITE  = colors.white
BLACK  = colors.black

# ── Style factory ─────────────────────────────────────────────────────────────
def S(name, **kw):
    s = ParagraphStyle(name, fontName='DV', fontSize=9.5,
                       textColor=BLACK, leading=14, spaceAfter=2)
    for k, v in kw.items(): setattr(s, k, v)
    return s

sT   = S('T',  fontName='DV-B',  fontSize=17, textColor=WHITE,
          alignment=TA_CENTER, leading=22, spaceAfter=0)
sST  = S('ST', fontName='DV',    fontSize=10, textColor=WHITE,
          alignment=TA_CENTER, leading=14, spaceAfter=0)
sH1  = S('H1', fontName='DV-B',  fontSize=11, textColor=WHITE,
          leading=15, spaceAfter=0, spaceBefore=0)
sH2  = S('H2', fontName='DV-B',  fontSize=10, textColor=NAVY,
          leading=14, spaceAfter=3, spaceBefore=7)
sBD  = S('BD', fontName='DV',    fontSize=9.5, leading=14, spaceAfter=3)
sFM  = S('FM', fontName='DVM',   fontSize=9,   leading=13, spaceAfter=1,
          leftIndent=6)
sNT  = S('NT', fontName='DV',    fontSize=8.5, textColor=DGRAY,
          leading=12, spaceAfter=2, leftIndent=6)
sTH  = S('TH', fontName='DV-B',  fontSize=8.5, textColor=WHITE,
          alignment=TA_CENTER)
sTD  = S('TD', fontName='DV',    fontSize=8.5, leading=12)
sTC  = S('TC', fontName='DVM',   fontSize=8,   leading=12)

# ── Helper constructors ───────────────────────────────────────────────────────
def pg(t, s=sBD):  return Paragraph(t, s)
def fm(t):         return Paragraph(t, sFM)
def nt(t):         return Paragraph(t, sNT)
def sp(h=5):       return Spacer(1, h)
def pb():          return PageBreak()

def section(title):
    """Navy header bar for a major section."""
    t = Table([[Paragraph(title, sH1)]], colWidths=[PW])
    t.setStyle(TableStyle([
        ('BACKGROUND',    (0,0),(-1,-1), NAVY),
        ('LEFTPADDING',   (0,0),(-1,-1), 8),
        ('RIGHTPADDING',  (0,0),(-1,-1), 8),
        ('TOPPADDING',    (0,0),(-1,-1), 5),
        ('BOTTOMPADDING', (0,0),(-1,-1), 5),
    ]))
    return t

def subsec(title):
    return Paragraph(f'<b>{title}</b>', sH2)

def eqbox(lines, shade=True):
    """Light-gray shaded equation block."""
    data = [[Paragraph(l, sFM)] for l in lines]
    t = Table(data, colWidths=[PW - 0.25*inch])
    ts = [
        ('LEFTPADDING',   (0,0),(-1,-1), 12),
        ('RIGHTPADDING',  (0,0),(-1,-1), 8),
        ('TOPPADDING',    (0,0),(-1,-1), 3),
        ('BOTTOMPADDING', (0,0),(-1,-1), 3),
        ('BOX',           (0,0),(-1,-1), 0.6, MGRAY),
    ]
    if shade:
        ts.append(('BACKGROUND', (0,0),(-1,-1), LGRAY))
    t.setStyle(TableStyle(ts))
    return t

def mrow(*entries, label='', col_w=None):
    """
    Render a matrix row-by-row with large brackets.
    entries: flat list of strings row-by-row, ncols inferred from col_w list length.
    col_w:   list of column widths for the inner matrix table.
    label:   string shown to the left of the matrix (e.g. 'ω̂ =').
    """
    ncols = len(col_w) if col_w else len(entries)
    nrows = len(entries) // ncols
    data  = []
    for r in range(nrows):
        row = [Paragraph(entries[r*ncols + c], sFM) for c in range(ncols)]
        data.append(row)

    if col_w is None:
        col_w = [0.7*inch] * ncols

    inner = Table(data, colWidths=col_w)
    inner.setStyle(TableStyle([
        ('ALIGN',         (0,0),(-1,-1), 'CENTER'),
        ('VALIGN',        (0,0),(-1,-1), 'MIDDLE'),
        ('TOPPADDING',    (0,0),(-1,-1), 2),
        ('BOTTOMPADDING', (0,0),(-1,-1), 2),
        ('LEFTPADDING',   (0,0),(-1,-1), 5),
        ('RIGHTPADDING',  (0,0),(-1,-1), 5),
    ]))

    # Scale bracket font size with number of rows
    bsz = 9 + nrows * 5
    lb  = Paragraph(f'<font size="{bsz}" name="DV">[</font>',
                     S(f'lb{nrows}', fontName='DV', fontSize=bsz,
                       alignment=TA_CENTER))
    rb  = Paragraph(f'<font size="{bsz}" name="DV">]</font>',
                     S(f'rb{nrows}', fontName='DV', fontSize=bsz,
                       alignment=TA_CENTER))

    bracket_tbl = Table([[lb, inner, rb]],
                         colWidths=[0.13*inch, None, 0.13*inch])
    bracket_tbl.setStyle(TableStyle([
        ('VALIGN',        (0,0),(-1,-1), 'MIDDLE'),
        ('TOPPADDING',    (0,0),(-1,-1), 0),
        ('BOTTOMPADDING', (0,0),(-1,-1), 0),
        ('LEFTPADDING',   (0,0),(-1,-1), 0),
        ('RIGHTPADDING',  (0,0),(-1,-1), 0),
    ]))

    if label:
        lbl = Paragraph(label, sFM)
        outer = Table([[lbl, bracket_tbl]],
                       colWidths=[len(label)*5.5, None])
        outer.setStyle(TableStyle([
            ('VALIGN',        (0,0),(-1,-1), 'MIDDLE'),
            ('LEFTPADDING',   (0,0),(-1,-1), 14),
            ('RIGHTPADDING',  (0,0),(-1,-1), 4),
            ('TOPPADDING',    (0,0),(-1,-1), 3),
            ('BOTTOMPADDING', (0,0),(-1,-1), 3),
        ]))
        return outer
    return bracket_tbl

# ── Unicode math shorthands ───────────────────────────────────────────────────
# These render correctly with DejaVu fonts
# ω̂ = ω + combining circumflex (U+0302)
# ġ = g + combining dot above  (U+0307)
# ṙ = r + combining dot above
# Ṙ = R + combining dot above

# ════════════════════════════════════════════════════════════════════════════
# BUILD STORY
# ════════════════════════════════════════════════════════════════════════════
story = []

# ── TITLE BLOCK ──────────────────────────────────────────────────────────────
title_tbl = Table([
    [Paragraph('ME 464  —  Kinematics Formula Sheet', sT)],
    [Paragraph('Chapter 2: Rigid Body Motion', sST)],
], colWidths=[PW])
title_tbl.setStyle(TableStyle([
    ('BACKGROUND',    (0,0),(-1,-1), NAVY),
    ('TOPPADDING',    (0,0),(-1,-1), 10),
    ('BOTTOMPADDING', (0,0),(-1,-1), 10),
    ('LEFTPADDING',   (0,0),(-1,-1), 8),
]))
story += [title_tbl, sp(10)]

# ════════════════════════════════════════════════════════════════════════════
# 2.1  RIGID MOTION
# ════════════════════════════════════════════════════════════════════════════
story += [section('2.1   Rigid Motion'), sp(4)]
story += [pg('A <b>rigid body motion</b> preserves all distances between points on the body. '
             'Its configuration is fully described by a rotation <b>R</b> and translation <b>p</b>.')]
story += [sp(3), subsec('Point Transformation  (body frame B  →  spatial frame A)')]
story += [eqbox(['p<sub>a</sub>  =  R<sub>ab</sub> p<sub>b</sub>  +  p<sub>ab</sub>']), sp(4)]

# ════════════════════════════════════════════════════════════════════════════
# 2.2  SO(3) — ROTATION MATRICES
# ════════════════════════════════════════════════════════════════════════════
story += [section('2.2   SO(3)  —  Rotation Matrices'), sp(4)]

story += [subsec('Definition')]
story += [eqbox([
    'SO(3)  =  { R ∈ ℝ<sup>3×3</sup>  :  R R<sup>T</sup> = I,    det(R) = +1 }',
]), sp(3)]

story += [subsec('Key Properties')]

prop_data = [
    [Paragraph('<b>Property</b>', sTH),      Paragraph('<b>Formula</b>', sTH)],
    [Paragraph('Orthogonality', sTD),        Paragraph('R R<sup>T</sup>  =  R<sup>T</sup> R  =  I', sTD)],
    [Paragraph('Inverse', sTD),              Paragraph('R<sup>−1</sup>  =  R<sup>T</sup>', sTD)],
    [Paragraph('Determinant', sTD),          Paragraph('det(R)  =  +1    (right-hand frames)', sTD)],
    [Paragraph('Closure', sTD),              Paragraph('R<sub>1</sub>, R<sub>2</sub> ∈ SO(3)  ⟹  R<sub>1</sub>R<sub>2</sub> ∈ SO(3)', sTD)],
    [Paragraph('Identity', sTD),             Paragraph('e  =  I<sub>3×3</sub>', sTD)],
    [Paragraph('Composition', sTD),          Paragraph('R<sub>ac</sub>  =  R<sub>ab</sub> R<sub>bc</sub>', sTD)],
    [Paragraph('Point / vector transform', sTD), Paragraph('v<sub>a</sub>  =  R<sub>ab</sub> v<sub>b</sub>', sTD)],
]
prop_tbl = Table(prop_data, colWidths=[1.8*inch, PW - 1.8*inch])
prop_tbl.setStyle(TableStyle([
    ('BACKGROUND',    (0,0),(-1,0),  NAVY),
    ('ROWBACKGROUNDS',(0,1),(-1,-1), [LGRAY, WHITE]),
    ('BOX',           (0,0),(-1,-1), 0.5, MGRAY),
    ('INNERGRID',     (0,0),(-1,-1), 0.3, MGRAY),
    ('LEFTPADDING',   (0,0),(-1,-1), 8),
    ('RIGHTPADDING',  (0,0),(-1,-1), 8),
    ('TOPPADDING',    (0,0),(-1,-1), 4),
    ('BOTTOMPADDING', (0,0),(-1,-1), 4),
    ('VALIGN',        (0,0),(-1,-1), 'MIDDLE'),
]))
story += [prop_tbl, sp(6)]

# ════════════════════════════════════════════════════════════════════════════
# 2.2.1b  HAT / VEE OPERATORS
# ════════════════════════════════════════════════════════════════════════════
story += [section('2.2.1b   Hat (∧) and Vee (∨) Operators'), sp(4)]

story += [subsec('Hat Operator   ω ∈ ℝ³  →  ω̂ ∈ so(3)   (3×1  →  3×3 skew-symmetric)')]
story += [sp(2)]
story += [mrow(
    '0',         '−ω<sub>3</sub>',  'ω<sub>2</sub>',
    'ω<sub>3</sub>',  '0',         '−ω<sub>1</sub>',
    '−ω<sub>2</sub>', 'ω<sub>1</sub>', '0',
    label='ω̂  =',
    col_w=[0.75*inch, 0.75*inch, 0.75*inch]
), sp(5)]
story += [eqbox([
    'Skew-symmetry:     ω̂<sup>T</sup>  =  −ω̂',
    'Cross product:     a × b  =  â b       (matrix-vector form of cross product)',
]), sp(3)]

story += [subsec('Vee Operator   (inverse of hat,  3×3  →  3×1)')]
story += [eqbox([
    'vee(ω̂)  =  ω         extraction:  ω  =  [ W<sub>3,2</sub> ;  W<sub>1,3</sub> ;  W<sub>2,1</sub> ]',
]), sp(3)]

story += [subsec('Useful Identities   (‖ω‖ = 1)')]
story += [eqbox([
    'ω̂²  =  ω ω<sup>T</sup>  −  I         (used to simplify Rodrigues and e<sup>ξ̂θ</sup>)',
    'ω̂³  =  −ω̂',
    'ω̂⁴  =  −ω̂²      ⟹     pattern repeats with period 2 after degree 2',
]), sp(4)]

# ════════════════════════════════════════════════════════════════════════════
# 2.2.2  RODRIGUES + MATRIX EXPONENTIAL
# ════════════════════════════════════════════════════════════════════════════
story += [section('2.2.2   Rodrigues\' Formula  &amp;  Exponential Coordinates'), sp(4)]

story += [subsec('Matrix Exponential  (Taylor series definition)')]
story += [eqbox([
    'e<sup>A</sup>  =  Σ<sub>k=0</sub><sup>∞</sup>  (1/k!) A<sup>k</sup>  '
    '=  I  +  A  +  A²/2!  +  A³/3!  +  ···',
    '',
    'Rotation via matrix exponential:    R(ω, θ)  =  e<sup>ω̂ θ</sup>',
    '(rotate about unit axis ω by angle θ)',
]), sp(3)]

story += [subsec('Rodrigues\' Formula   (closed form,  ‖ω‖ = 1)')]
story += [eqbox([
    'R(ω, θ)  =  e<sup>ω̂ θ</sup>  =  I  +  sin(θ) ω̂  +  (1 − cos θ) ω̂²',
]), sp(3)]

story += [subsec('Lie Algebra  so(3)')]
story += [eqbox([
    'so(3)  =  { S ∈ ℝ<sup>3×3</sup>  :  S<sup>T</sup> = −S }         (all 3×3 skew-symmetric matrices)',
    'ω̂ ∈ so(3)    for any ω ∈ ℝ³',
]), sp(3)]

story += [subsec('Inverse Rodrigues   (given R, recover ω and θ)')]
story += [eqbox([
    'θ  =  arccos( ( trace(R) − 1 ) / 2 )',
    'ω  =  vee( R − R<sup>T</sup> )  /  ( 2 sin θ )         valid for θ ≠ 0, π',
]), sp(4)]

# ════════════════════════════════════════════════════════════════════════════
# 2.3  SE(3) — HOMOGENEOUS TRANSFORMATIONS
# ════════════════════════════════════════════════════════════════════════════
story += [pb()]   # new page
story += [section('2.3   SE(3)  —  Homogeneous Geometric Transformations (HGT)'), sp(4)]

story += [pg('The <b>HGT</b> combines rotation <i>R</i> and position <i>p</i> into a single 4×4 matrix '
             'that acts on homogeneous coordinates:')]
story += [sp(4)]
story += [mrow(
    'R<sub>ab</sub>', 'p<sub>ab</sub>',
    '0  0  0',       '1',
    label='g<sub>ab</sub>  =',
    col_w=[1.1*inch, 0.65*inch]
), sp(5)]
story += [eqbox([
    'SE(3)  =  { (R, p) : R ∈ SO(3),  p ∈ ℝ³ }',
    '',
    'Point transform:  p<sub>a</sub>  =  g<sub>ab</sub> [p<sub>b</sub> ; 1]  '
    '=  R<sub>ab</sub> p<sub>b</sub>  +  p<sub>ab</sub>',
]), sp(3)]

story += [subsec('HGT Inverse   (analytic — never use matrix inversion)')]
story += [sp(2)]
story += [mrow(
    'R<sup>T</sup>',  '−R<sup>T</sup>p',
    '0  0  0',        '1',
    label='g<sup>−1</sup>  =',
    col_w=[0.8*inch, 1.0*inch]
), sp(5)]

story += [subsec('Group Properties of SE(3)')]
se3_data = [
    [Paragraph('<b>Property</b>', sTH),  Paragraph('<b>Formula</b>', sTH)],
    [Paragraph('Closure', sTD),          Paragraph('g<sub>1</sub>, g<sub>2</sub> ∈ SE(3)  ⟹  g<sub>1</sub> g<sub>2</sub> ∈ SE(3)', sTD)],
    [Paragraph('Identity', sTD),         Paragraph('e  =  [ I,  0 ; 0,  1 ]  (4×4)', sTD)],
    [Paragraph('Inverse', sTD),          Paragraph('g<sup>−1</sup>  =  [ R<sup>T</sup>,  −R<sup>T</sup>p ; 0,  1 ]', sTD)],
    [Paragraph('Composition', sTD),      Paragraph('g<sub>ac</sub>  =  g<sub>ab</sub> g<sub>bc</sub>', sTD)],
    [Paragraph('Associativity', sTD),    Paragraph('(g<sub>1</sub> g<sub>2</sub>) g<sub>3</sub>  =  g<sub>1</sub> (g<sub>2</sub> g<sub>3</sub>)', sTD)],
]
se3_tbl = Table(se3_data, colWidths=[1.5*inch, PW - 1.5*inch])
se3_tbl.setStyle(TableStyle([
    ('BACKGROUND',    (0,0),(-1,0),  NAVY),
    ('ROWBACKGROUNDS',(0,1),(-1,-1), [LGRAY, WHITE]),
    ('BOX',           (0,0),(-1,-1), 0.5, MGRAY),
    ('INNERGRID',     (0,0),(-1,-1), 0.3, MGRAY),
    ('LEFTPADDING',   (0,0),(-1,-1), 8),
    ('RIGHTPADDING',  (0,0),(-1,-1), 8),
    ('TOPPADDING',    (0,0),(-1,-1), 4),
    ('BOTTOMPADDING', (0,0),(-1,-1), 4),
    ('VALIGN',        (0,0),(-1,-1), 'MIDDLE'),
]))
story += [se3_tbl, sp(6)]

# ════════════════════════════════════════════════════════════════════════════
# 2.3.2a  TWISTS
# ════════════════════════════════════════════════════════════════════════════
story += [section('2.3.2a   Twists  —  Exponential Coordinates for Rigid Motion'), sp(4)]

# Two-column layout: revolute | prismatic
left_rev = [
    Paragraph('<b>Revolute Joint</b>  (rotation about ω through q)', sH2),
    Paragraph('Given unit axis ω  (‖ω‖ = 1)  and point q on axis:', sBD),
    sp(3),
    eqbox([
        'v  =  −ω × q  =  −ω̂ q',
        '',
        'ξ  =  [ v ; ω ]   (6×1)',
    ]),
]
right_pris = [
    Paragraph('<b>Prismatic Joint</b>  (pure translation along v)', sH2),
    Paragraph('Given unit direction v  (‖v‖ = 1):', sBD),
    sp(3),
    eqbox([
        'ξ  =  [ v ; 0 ]   (6×1)',
        '',
        '',
    ]),
]
two_col = Table([[left_rev, right_pris]],
                colWidths=[PW/2 - 0.1*inch, PW/2 - 0.1*inch])
two_col.setStyle(TableStyle([
    ('VALIGN',       (0,0),(-1,-1), 'TOP'),
    ('LEFTPADDING',  (0,0),(-1,-1), 0),
    ('RIGHTPADDING', (0,0),(-1,-1), 0),
    ('TOPPADDING',   (0,0),(-1,-1), 0),
    ('BOTTOMPADDING',(0,0),(-1,-1), 0),
    ('LINEAFTER',    (0,0),(0,-1),  0.5, MGRAY),
]))
story += [two_col, sp(5)]

story += [subsec('Wedge (∧) Operator   6×1  →  4×4 twist matrix')]
story += [sp(2)]
story += [mrow(
    'ω̂',   'v',
    '0 0 0', '0',
    label='ξ̂  =  [v ; ω]<sup>∧</sup>  =',
    col_w=[0.9*inch, 0.5*inch]
), sp(5)]

story += [subsec('Vee (∨) Operator   4×4  →  6×1   (inverse of wedge)')]
story += [eqbox([
    'vee(ξ̂)  =  ξ  =  [ v ; ω ]         extracts  v = ξ̂(1:3, 4),  ω = vee(ξ̂(1:3, 1:3))',
]), sp(3)]

story += [subsec('Forward Kinematics   (product of exponentials, spatial frame)')]
story += [eqbox([
    'g<sub>ab</sub>(θ)  =  e<sup>ξ̂ θ</sup>  g<sub>ab</sub>(0)',
    '',
    'p<sub>a</sub>(θ)  =  g<sub>ab</sub>(θ) [p<sub>b</sub> ; 1]',
]), sp(4)]

# ════════════════════════════════════════════════════════════════════════════
# 2.3.2b  MATRIX EXPONENTIAL FORMULAS
# ════════════════════════════════════════════════════════════════════════════
story += [section('2.3.2b   Matrix Exponential Formulas for Twists'), sp(4)]

story += [subsec('Revolute Joint   (‖ω‖ = 1)')]
story += [sp(2)]
story += [mrow(
    'R',       'p',
    '0  0  0', '1',
    label='e<sup>ξ̂ θ</sup>  =',
    col_w=[0.7*inch, 0.55*inch]
), sp(5)]
story += [eqbox([
    'R  =  e<sup>ω̂ θ</sup>  =  I  +  sin(θ) ω̂  +  (1 − cos θ) ω̂²          (Rodrigues)',
    '',
    'p  =  (I − R) ω̂ v                                  (when ω<sup>T</sup>v = 0,  i.e. revolute joint)',
    '',
    'p  =  ( Iθ  +  (1−cos θ) ω̂  +  (θ−sin θ) ω̂² ) v    (general formula)',
]), sp(4)]

story += [subsec('Prismatic Joint   (ω = 0)')]
story += [sp(2)]
story += [mrow(
    'I',       'v θ',
    '0  0  0', '1',
    label='e<sup>ξ̂ θ</sup>  =',
    col_w=[0.5*inch, 0.6*inch]
), sp(5)]

# ════════════════════════════════════════════════════════════════════════════
# 2.4  RIGID BODY VELOCITY
# ════════════════════════════════════════════════════════════════════════════
story += [pb()]
story += [section('2.4   Rigid Body Velocity'), sp(4)]

story += [subsec('2.4.1   Rotational Velocity')]
story += [pg('Differentiating R R<sup>T</sup> = I gives  Ṙ R<sup>T</sup>  =  −(Ṙ R<sup>T</sup>)<sup>T</sup>  '
             '→  Ṙ R<sup>T</sup>  is skew-symmetric.', sBD)]
story += [sp(3)]
story += [eqbox([
    'Spatial angular velocity:    ω̂<sup>s</sup><sub>ab</sub>  :=  Ṙ<sub>ab</sub> R<sub>ab</sub><sup>T</sup>  '
    '=  Ṙ<sub>ab</sub> R<sub>ab</sub><sup>−1</sup>',
    '',
    'Body angular velocity:       ω̂<sup>b</sup><sub>ab</sub>  :=  R<sub>ab</sub><sup>T</sup> Ṙ<sub>ab</sub>  '
    '=  R<sub>ab</sub><sup>−1</sup> Ṙ<sub>ab</sub>',
    '',
    'Relation (matrix):           ω̂<sup>b</sup>  =  R<sup>−1</sup> ω̂<sup>s</sup> R',
    'Relation (vector):           ω<sup>b</sup>   =  R<sup>−1</sup> ω<sup>s</sup>',
    '',
    'Point velocity (spatial):    v<sub>q</sub>  =  ω<sup>s</sup> × q<sub>a</sub>  =  ω̂<sup>s</sup> q<sub>a</sub>',
    'Point velocity (body):       v<sub>q</sub>  =  ω<sup>b</sup> × q<sub>b</sub>  =  ω̂<sup>b</sup> q<sub>b</sub>',
]), sp(4)]

story += [subsec('2.4.2a   Spatial and Body Velocity   (g ∈ SE(3))')]
story += [eqbox([
    'Spatial velocity (4×4 matrix):   V̂<sup>s</sup><sub>ab</sub>  =  ġ<sub>ab</sub> g<sub>ab</sub><sup>−1</sup>',
    'Body velocity (4×4 matrix):      V̂<sup>b</sup><sub>ab</sub>  =  g<sub>ab</sub><sup>−1</sup> ġ<sub>ab</sub>',
    '',
    'Spatial velocity (6×1 vector):   V<sup>s</sup>  =  vee( V̂<sup>s</sup> )  =  [ v<sup>s</sup> ; ω<sup>s</sup> ]',
    'Body velocity (6×1 vector):      V<sup>b</sup>  =  vee( V̂<sup>b</sup> )  =  [ v<sup>b</sup> ; ω<sup>b</sup> ]',
]), sp(4)]

story += [subsec('2.4.2b   Adjoint Transformation   Ad<sub>g</sub> ∈ ℝ<sup>6×6</sup>')]
story += [pg('For g = [R, p; 0, 1]  ∈  SE(3), the <b>Adjoint</b> is:')]
story += [sp(3)]
story += [mrow(
    'R',    'p̂ R',
    '0 0 0', 'R',
    label='Ad<sub>g</sub>  =',
    col_w=[0.75*inch, 0.9*inch]
), sp(5)]
story += [eqbox([
    'where  p̂ = hat(p)  is the 3×3 skew-symmetric matrix of p',
    '',
    'Spatial ↔ Body:    V<sup>s</sup>  =  Ad<sub>g<sub>ab</sub></sub>  V<sup>b</sup>',
    '4×4 form:          V̂<sup>s</sup>  =  g V̂<sup>b</sup> g<sup>−1</sup>',
    'Twist transform:   ξ′  =  Ad<sub>g</sub> ξ        (or  ξ̂′ = g ξ̂ g<sup>−1</sup>  in 4×4)',
]), sp(4)]

story += [subsec('2.4.3   Velocity via Twists  —  Single Joint')]
story += [eqbox([
    'For  g(θ) = e<sup>ξ̂ θ</sup> g(0):',
    '',
    'Spatial velocity:   V<sup>s</sup>  =  ξ θ̇         (does NOT depend on θ)',
    'Body velocity:      V<sup>b</sup>  =  Ad<sub>g<sup>−1</sup></sub> ξ θ̇',
]), sp(3)]
story += [nt('Key insight: for a single revolute joint using the spatial formulation, V<sup>s</sup> = ξ θ̇ '
             'is a constant twist scaled by joint velocity.')]
story += [sp(3)]

story += [subsec('Velocity of a Body Point  r  at position r<sub>a</sub> in the spatial frame')]
story += [eqbox([
    'ṙ<sub>a</sub>  =  v<sup>s</sup>  +  ω<sup>s</sup> × r<sub>a</sub>',
    '',
    'where  v<sup>s</sup> = V<sup>s</sup>(1:3)   (linear part)     '
    'and  ω<sup>s</sup> = V<sup>s</sup>(4:6)   (angular part)',
    '',
    'Matrix form:  [ ṙ<sub>a</sub> ; 0 ]  =  V̂<sup>s</sup> [ r<sub>a</sub> ; 1 ]  '
    '=  [ ω̂<sup>s</sup>  v<sup>s</sup> ; 0  0 ] [ r<sub>a</sub> ; 1 ]',
]), sp(4)]

story += [subsec('2.4.4   Velocity Composition   (three frames A, B, C)')]
story += [eqbox([
    'g<sub>ac</sub>  =  g<sub>ab</sub>  g<sub>bc</sub>',
    '',
    'V<sup>s</sup><sub>ac</sub>  =  V<sup>s</sup><sub>ab</sub>  +  Ad<sub>g<sub>ab</sub></sub>  V<sup>s</sup><sub>bc</sub>',
]), sp(4)]

# ════════════════════════════════════════════════════════════════════════════
# 2.5  WRENCHES
# ════════════════════════════════════════════════════════════════════════════
story += [section('2.5   Wrenches'), sp(4)]

story += [pg('A <b>wrench</b> F is a 6×1 vector combining force f and torque τ. '
             'It is the dual of a twist under the power pairing:')]
story += [sp(3)]
story += [eqbox([
    'F  =  [ f ; τ ]        f ∈ ℝ³  (force),    τ ∈ ℝ³  (torque / moment)',
    '',
    'Power  P  =  F<sup>T</sup> V  =  f · v  +  τ · ω        (frame-independent scalar)',
]), sp(3)]

story += [subsec('Wrench Transformation   (power duality  F<sup>a</sup> · V<sup>a</sup>  =  F<sup>b</sup> · V<sup>b</sup>)')]
story += [eqbox([
    'Since  V<sup>a</sup>  =  Ad<sub>g<sub>ab</sub></sub>  V<sup>b</sup>,  wrenches must satisfy:',
    '',
    'F<sup>b</sup>  =  Ad<sub>g<sub>ab</sub></sub><sup>T</sup>  F<sup>a</sup>',
]), sp(4)]

# ════════════════════════════════════════════════════════════════════════════
# MATLAB FUNCTION TABLE
# ════════════════════════════════════════════════════════════════════════════
story += [pb()]
story += [section('MATLAB Function Library Reference'), sp(4)]

fn_rows = [
    [Paragraph('<b>Function Call</b>', sTH),
     Paragraph('<b>Operation</b>', sTH),
     Paragraph('<b>Signature</b>', sTH)],
    [Paragraph('hat_PCM(ω)', sTC),
     Paragraph('Hat operator  ω̂  (3×3 skew-symmetric)', sTD),
     Paragraph('3×1  →  3×3', sTD)],
    [Paragraph('vee_PCM(W)', sTC),
     Paragraph('Vee operator  (inverse hat)', sTD),
     Paragraph('3×3  →  3×1', sTD)],
    [Paragraph('expr_PCM(ω, θ)', sTC),
     Paragraph('Rodrigues rotation  R = e<sup>ω̂θ</sup>', sTD),
     Paragraph('ω, θ  →  3×3 R', sTD)],
    [Paragraph('twistr_PCM(ω, q)', sTC),
     Paragraph('Revolute twist  ξ = [−ω̂q ; ω]', sTD),
     Paragraph('ω, q  →  6×1 ξ', sTD)],
    [Paragraph('twistp_PCM(v)', sTC),
     Paragraph('Prismatic twist  ξ = [v ; 0]', sTD),
     Paragraph('v  →  6×1 ξ', sTD)],
    [Paragraph('wedge_PCM(ξ)', sTC),
     Paragraph('Wedge operator  ξ → ξ̂', sTD),
     Paragraph('6×1  →  4×4', sTD)],
    [Paragraph('veeTwist_PCM(Ξ)', sTC),
     Paragraph('Vee twist operator  ξ̂ → ξ', sTD),
     Paragraph('4×4  →  6×1', sTD)],
    [Paragraph('expt_PCM(ξ, θ)', sTC),
     Paragraph('HGT exponential — revolute  e<sup>ξ̂θ</sup>', sTD),
     Paragraph('ξ, θ  →  4×4 g', sTD)],
    [Paragraph('extp_PCM(ξ, θ)', sTC),
     Paragraph('HGT exponential — prismatic  [I, vθ; 0,1]', sTD),
     Paragraph('ξ, θ  →  4×4 g', sTD)],
    [Paragraph('hgt_PCM(R, p)', sTC),
     Paragraph('Build HGT from R and p:  [R, p; 0, 1]', sTD),
     Paragraph('R, p  →  4×4 g', sTD)],
    [Paragraph('invHGT_PCM(g)', sTC),
     Paragraph('Analytic HGT inverse:  [R<sup>T</sup>, −R<sup>T</sup>p ; 0, 1]', sTD),
     Paragraph('4×4  →  4×4', sTD)],
    [Paragraph('adjoint_PCM(g)', sTC),
     Paragraph('Adjoint:  Ad<sub>g</sub> = [R, p̂R ; 0, R]', sTD),
     Paragraph('4×4  →  6×6', sTD)],
    [Paragraph('spatialVel_PCM(g, ġ)', sTC),
     Paragraph('Spatial velocity:  vee(ġ g<sup>−1</sup>)', sTD),
     Paragraph('g, ġ  →  6×1 V<sup>s</sup>', sTD)],
    [Paragraph('bodyVel_PCM(g, ġ)', sTC),
     Paragraph('Body velocity:  vee(g<sup>−1</sup> ġ)', sTD),
     Paragraph('g, ġ  →  6×1 V<sup>b</sup>', sTD)],
    [Paragraph('wrenchTransform_PCM(g, F)', sTC),
     Paragraph('Wrench transform:  Ad<sub>g</sub><sup>T</sup> F', sTD),
     Paragraph('g, F<sup>a</sup>  →  6×1 F<sup>b</sup>', sTD)],
]
fn_cw = [1.95*inch, PW - 3.45*inch, 1.5*inch]
fn_tbl = Table(fn_rows, colWidths=fn_cw)
fn_tbl.setStyle(TableStyle([
    ('BACKGROUND',    (0,0),(-1,0),  NAVY),
    ('ROWBACKGROUNDS',(0,1),(-1,-1), [LGRAY, WHITE]),
    ('BOX',           (0,0),(-1,-1), 0.5, MGRAY),
    ('INNERGRID',     (0,0),(-1,-1), 0.3, MGRAY),
    ('LEFTPADDING',   (0,0),(-1,-1), 7),
    ('RIGHTPADDING',  (0,0),(-1,-1), 7),
    ('TOPPADDING',    (0,0),(-1,-1), 4),
    ('BOTTOMPADDING', (0,0),(-1,-1), 4),
    ('VALIGN',        (0,0),(-1,-1), 'MIDDLE'),
]))
story += [fn_tbl, sp(10)]

# ════════════════════════════════════════════════════════════════════════════
# QUICK REFERENCE — KEY IDENTITIES (blue-tinted box)
# ════════════════════════════════════════════════════════════════════════════
story += [section('Quick Reference  —  Key Identities'), sp(4)]

qr_lines = [
    '<b>Inverses</b>',
    '    R<sup>−1</sup>  =  R<sup>T</sup>',
    '    g<sup>−1</sup>  =  [ R<sup>T</sup>,  −R<sup>T</sup>p  ;  0,  1 ]',
    '',
    '<b>Rodrigues &amp; Hat Identities</b>  (‖ω‖ = 1)',
    '    R(ω,θ)  =  I + sin(θ)ω̂ + (1−cos θ)ω̂²',
    '    ω̂²  =  ωω<sup>T</sup> − I          ω̂³  =  −ω̂',
    '',
    '<b>Revolute Twist</b>',
    '    v  =  −ω × q  =  −ω̂q          ξ  =  [ v ; ω ]',
    '',
    '<b>Matrix Exponentials</b>',
    '    e<sup>ξ̂θ</sup>  =  [ R,  (I−R)ω̂v ; 0, 1 ]     revolute   (ω<sup>T</sup>v = 0)',
    '    e<sup>ξ̂θ</sup>  =  [ I,   vθ      ; 0, 1 ]     prismatic',
    '',
    '<b>Angular Velocity</b>',
    '    ω̂<sup>s</sup> = Ṙ R<sup>T</sup>                ω̂<sup>b</sup> = R<sup>T</sup> Ṙ',
    '',
    '<b>Rigid Body Velocity</b>',
    '    V̂<sup>s</sup> = ġ g<sup>−1</sup>               V̂<sup>b</sup> = g<sup>−1</sup> ġ',
    '    V<sup>s</sup>  =  Ad<sub>g</sub> V<sup>b</sup>',
    '    V<sup>s</sup>  =  ξ θ̇             (single joint, spatial frame)',
    '',
    '<b>Point Velocity</b>',
    '    ṙ<sub>a</sub>  =  v<sup>s</sup>  +  ω<sup>s</sup> × r<sub>a</sub>',
    '',
    '<b>Velocity Composition</b>',
    '    V<sup>s</sup><sub>ac</sub>  =  V<sup>s</sup><sub>ab</sub>  +  Ad<sub>g<sub>ab</sub></sub> V<sup>s</sup><sub>bc</sub>',
    '',
    '<b>Wrenches</b>',
    '    F  =  [ f ; τ ]          P  =  F<sup>T</sup>V  =  f·v + τ·ω',
    '    F<sup>b</sup>  =  Ad<sub>g<sub>ab</sub></sub><sup>T</sup> F<sup>a</sup>',
]

qr_data = [[Paragraph(l, sFM)] for l in qr_lines]
qr_tbl  = Table(qr_data, colWidths=[PW - 0.3*inch])
qr_tbl.setStyle(TableStyle([
    ('BACKGROUND',    (0,0),(-1,-1), LBLUE),
    ('BOX',           (0,0),(-1,-1), 1.2, DBLUE),
    ('LEFTPADDING',   (0,0),(-1,-1), 14),
    ('RIGHTPADDING',  (0,0),(-1,-1), 10),
    ('TOPPADDING',    (0,0),(-1,-1), 3),
    ('BOTTOMPADDING', (0,0),(-1,-1), 3),
]))
story += [qr_tbl]

# ── Build PDF ────────────────────────────────────────────────────────────────
doc.build(story)
print(f'✓  PDF written → {OUT}')
