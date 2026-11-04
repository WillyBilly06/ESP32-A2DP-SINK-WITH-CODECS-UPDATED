"""Generate three bounded SBC arithmetic port units; never modify SDK sources."""
import argparse
import hashlib
import json
from pathlib import Path
import re


def generate(source, output):
    output.mkdir(parents=True, exist_ok=True)
    receipt = {}
    for name in ["synthesis-8-generated.c", "synthesis-dct8.c", "synthesis-sbc.c"]:
        raw = (source/name).read_bytes()
        text = raw.decode("utf-8")
        if name == "synthesis-8-generated.c":
            text, count = re.subn(r'\(MUL_16S_16S\(([^)\n]+)\)\) << ([1-5])',
                                  r'DBT_SBC_LSL32(MUL_16S_16S(\1), \2)', text)
            if count != 27:
                raise RuntimeError("Unexpected SBC synthesis source; review shift port: " + str(count))
        else:
            replacements = {
                "synthesis-dct8.c": {
                    "y = x - (y<<1);": "y = x - DBT_SBC_LSL32(y,1);",
                    "(MUL_32S_32S_HI(K,x)<<2)": "DBT_SBC_LSL32(MUL_32S_32S_HI(K,x),2)"},
                "synthesis-sbc.c": {
                    "(MUL_16S_32S_HI(K, sample)<<2)": "DBT_SBC_LSL32(MUL_16S_32S_HI(K,sample),2)"}}[name]
            count = len(replacements)
            for before, after in replacements.items():
                if text.count(before) != 1:
                    raise RuntimeError("Unexpected SBC source; review arithmetic port: " + name)
                text = text.replace(before, after)
        include = '#include <oi_codec_sbc_private.h>' if '<oi_codec_sbc_private.h>' in text else '#include "oi_codec_sbc_private.h"'
        if text.count(include) != 1:
            raise RuntimeError("Unexpected SBC private include")
        text = text.replace(include, include + '\n/* Defined 32-bit left shifts; original license and tables retained. */\n'
            '#define DBT_SBC_LSL32(value,bits) ((OI_INT32)((OI_UINT32)(value) << (bits)))\n')
        data = text.encode("utf-8")
        if len(data) > 65536:
            raise RuntimeError("Unexpected generated SBC unit size")
        (output/name).write_bytes(data)
        receipt[name] = {"source_sha256": hashlib.sha256(raw).hexdigest(),
                         "generated_sha256": hashlib.sha256(data).hexdigest(), "shift_sites": count}
    (output/"port.json").write_text(json.dumps(receipt, indent=2)+"\n")
    return receipt


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(generate(args.source, args.output), indent=2))
