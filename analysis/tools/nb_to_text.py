#!/usr/bin/env python3
"""Export Mathematica notebooks (.nb) to plain text, without Mathematica.

GitHub cannot display .nb files, so every notebook in analysis/mathematica/ has
a text copy in analysis/mathematica/text/ with its input cells in Mathematica
syntax (and the stored results of the numeric notebooks).

    python3 analysis/tools/nb_to_text.py            # export all notebooks
    python3 analysis/tools/nb_to_text.py file.nb    # print one notebook

How it works: a notebook is a Wolfram expression Notebook[{Cell[BoxData[...],
"Input"], ...}]. The script tokenizes and parses that expression, then turns
the typeset "boxes" (RowBox, FractionBox, SuperscriptBox, ...) back into
linear Mathematica input such as (a1^2 m1)/(12).
"""
import os
import re
import sys

# Named characters \[Name] -> text
SPECIAL = {
    'Theta': 'θ', 'Alpha': 'α', 'Beta': 'β', 'Gamma': 'γ', 'Tau': 'τ', 'Omega': 'ω', 'Pi': 'π',
    'Phi': 'φ', 'Psi': 'ψ', 'Delta': 'δ', 'Mu': 'μ', 'Rho': 'ρ', 'Sigma': 'σ', 'Lambda': 'λ',
    'IndentingNewLine': '\n', 'NoBreak': '', 'InvisibleSpace': '', 'InvisibleTimes': '',
    'Times': '*', 'Rule': '->', 'Transpose': 'ᵀ', 'Degree': ' Degree', 'Prime': "'",
    'Equal': '==', 'LeftDoubleBracket': '[[', 'RightDoubleBracket': ']]', 'Cross': '×',
    'PartialD': '∂', 'Infinity': '∞', 'LessEqual': '<=', 'GreaterEqual': '>=', 'NotEqual': '!=',
}
WRAPPERS = {'TagBox', 'StyleBox', 'FormBox', 'InterpretationBox', 'TemplateBox', 'ButtonBox',
            'DynamicBox', 'FrameBox', 'AdjustmentBox', 'UnderscriptBox', 'UnderoverscriptBox',
            'BoxData', 'TextData'}


def tokenize(src):
    """Yield (kind, value) tokens of a Wolfram expression."""
    i, n = 0, len(src)
    while i < n:
        c = src[i]
        if c.isspace():
            i += 1
        elif c == '"':
            j, buf = i + 1, []
            while src[j] != '"':
                if src[j] == '\\' and src[j + 1] == '[':
                    k = src.index(']', j)
                    buf.append(SPECIAL.get(src[j + 2:k], src[j + 2:k]))
                    j = k + 1
                elif src[j] == '\\' and src[j + 1] in '<>':
                    j += 2
                elif src[j] == '\\':
                    buf.append(src[j + 1])
                    j += 2
                else:
                    buf.append(src[j])
                    j += 1
            yield 'str', ''.join(buf)
            i = j + 1
        elif c in '[]{},':
            yield c, c
            i += 1
        elif src.startswith('->', i) or src.startswith(':>', i):
            yield 'rule', '->'
            i += 2
        else:
            tok = re.match(r'[^\s\[\]{},"]+', src[i:]).group(0)
            yield 'sym', tok
            i += len(tok)


def parse(tokens):
    """Parse tokens into nested tuples: ('str'|'sym', v), ('list', items), ('call', head, args)."""
    tokens = list(tokens)
    pos = 0

    def expr():
        nonlocal pos
        kind, val = tokens[pos]
        pos += 1
        if kind == '{':
            items = []
            while tokens[pos][0] != '}':
                items.append(expr())
                pos += tokens[pos][0] == ','
            pos += 1
            node = ('list', items)
        elif kind == 'sym' and pos < len(tokens) and tokens[pos][0] == '[':
            pos += 1
            args = []
            while tokens[pos][0] != ']':
                args.append(expr())
                pos += tokens[pos][0] == ','
            pos += 1
            node = ('call', val, args)
        else:
            node = (kind, val)
        while pos < len(tokens) and tokens[pos][0] == 'rule':   # options like Key -> value
            pos += 1
            node = ('rule', node, expr())
        return node

    return expr()


def boxes_to_text(node):
    """Typeset boxes -> linear Mathematica input."""
    kind = node[0]
    if kind in ('str', 'sym'):
        return node[1]
    if kind == 'list':
        return ''.join(boxes_to_text(x) for x in node[1])
    if kind == 'rule':
        return ''
    head, args = node[1], [a for a in node[2] if a[0] != 'rule']
    t = [boxes_to_text(a) for a in args]
    if head == 'RowBox':
        return t[0]
    if head == 'SuperscriptBox':
        return f'{t[0]}^{t[1]}' if len(t[1]) == 1 else f'{t[0]}^({t[1]})'
    if head == 'SubscriptBox':
        return f'{t[0]}_{t[1]}'
    if head == 'FractionBox':
        return f'({t[0]})/({t[1]})'
    if head == 'SqrtBox':
        return f'Sqrt[{t[0]}]'
    if head == 'GridBox':   # matrices
        return '{' + ', '.join('{' + ', '.join(boxes_to_text(c) for c in row[1]) + '}'
                               for row in args[0][1]) + '}'
    if head in WRAPPERS or head == 'OverscriptBox':
        return t[0] if t else ''
    return ''


def cells(root):
    """All (style, content) pairs of the notebook, in reading order."""
    stack = [root]
    while stack:
        node = stack.pop()
        if node[0] == 'call' and node[1] == 'Cell':
            args = [a for a in node[2] if a[0] != 'rule']
            if len(args) >= 2 and args[1][0] == 'str':
                yield args[1][1], args[0]
            if args and args[0][0] == 'call' and args[0][1] == 'CellGroupData':
                stack.extend(reversed(args[0][2][0][1]))
        elif node[0] == 'call':
            stack.extend(reversed(node[2]))
        elif node[0] == 'list':
            stack.extend(reversed(node[1]))


def notebook_to_text(path, styles=('Title', 'Section', 'Text', 'Input', 'Output')):
    src = open(path, encoding='utf-8', errors='replace').read()
    body = src[src.index('Notebook['):src.rindex('(* End of Notebook Content *)')]
    parts = []
    for style, content in cells(parse(tokenize(body))):
        if style not in styles:
            continue
        text = boxes_to_text(content).strip()
        # Keep the stored results short (plots and huge expressions are skipped).
        if style == 'Output' and (not text or len(text) > 300):
            continue
        if text:
            parts.append(f'(* ---- {style} ---- *)\n{text}\n')
    return '\n'.join(parts)


def main():
    if len(sys.argv) > 1:
        print(notebook_to_text(sys.argv[1]))
        return
    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'mathematica')
    out_dir = os.path.join(folder, 'text')
    os.makedirs(out_dir, exist_ok=True)
    for name in sorted(os.listdir(folder)):
        if name.endswith('.nb'):
            text = notebook_to_text(os.path.join(folder, name))
            with open(os.path.join(out_dir, name[:-3] + '.txt'), 'w') as fh:
                fh.write(f'(* Text export of {name} (analysis/tools/nb_to_text.py) *)\n\n{text}')
            print(f'{name}: {len(text)} characters')


if __name__ == '__main__':
    main()
