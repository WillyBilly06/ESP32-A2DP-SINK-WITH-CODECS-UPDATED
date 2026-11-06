"""Generate source-pinned Helix channel and side-information guards.
Original SDK units and their license notices remain unchanged.
"""
import argparse
import hashlib
import json
from pathlib import Path

PINS = {
    "aacdec.c": "f3260aeca52c91675761b8dccf2ec83cf42f8cc07ef47da10676ec1aa66ea5d6",
    "noiseless.c": "605d533560e577a2e3c68e0957808a92904ca48127a093f203cfe63014761203",
    "decelmnt.c": "7f8703a2442629f7a798ebc5afb7037011d98c04129bfcd5110ff81621534d7b",
    "huffman.c": "a740d7a42d414a707c8268d902c01b2c219a91c785e15a477806a733a0625e4c",
}

def replace_once(text, before, after):
    if text.count(before) != 1:
        raise RuntimeError("Unexpected Helix guard site: " + before[:80])
    return text.replace(before, after)

def replace_function(text, signature, replacement):
    if text.count(signature) != 1:
        raise RuntimeError("Unexpected Helix function: " + signature)
    start = text.index(signature)
    end = text.index("\n}\n", start) + 3
    return text[:start] + replacement + text[end:]

def guard_noiseless(text):
    text = replace_once(text, '#include "coder.h"', '#include "coder.h"\n#include "helix_internal_guards.h"')
    text = replace_function(text, "static void DecodeSectionData(", """static int DecodeSectionData(BitStreamInfo *bsi, int winSequence, int numWinGrp, int maxSFB, unsigned char *sfbCodeBook)
{
    int sectLenBits = winSequence == 2 ? 3 : 5;
    int escape = (1 << sectLenBits) - 1;
    if(numWinGrp < 1 || numWinGrp > MAX_WIN_GROUPS || maxSFB < 0 ||
       maxSFB > MAX_SF_BANDS || numWinGrp * maxSFB > MAX_SF_BANDS)return -1;
    for(int g = 0; g < numWinGrp; ++g) {
        int sfb = 0;
        while(sfb < maxSFB) {
            if(dbt_helix_bits_left(bsi) < 4)return -1;
            int cb = GetBits(bsi, 4), length = 0, part;
            if(cb == 12)return -1;
            do {
                if(dbt_helix_bits_left(bsi) < sectLenBits)return -1;
                part = GetBits(bsi, sectLenBits);
                if(part > maxSFB - sfb - length)return -1;
                length += part;
            } while(part == escape);
            /* Every accepted section advances and fits before any write. */
            if(!length)return -1;
            sfb += length;
            for(int i = 0; i < length; ++i)*sfbCodeBook++ = (unsigned char)cb;
        }
    }
    return 0;
}
""")
    text = replace_once(text, "static void DecodeTNSInfo(", "static int DecodeTNSInfo(")
    for indent in ("\t\t\t\t", "\t\t\t"):
        needle = "\n" + indent + "if (*filtOrder) {"
        guard = "\n" + indent + "if (*filtOrder > MAX_TNS_ORDER || tnsCoef - ti->coef > MAX_TNS_COEFS - *filtOrder)\n" + indent + "\treturn -1;"
        text = replace_once(text, needle, guard + needle)
    end = text.index("\n}\n", text.index("static int DecodeTNSInfo("))
    text = text[:end] + "\n\treturn dbt_helix_bits_left(bsi) < 0 ? -1 : 0;" + text[end:]
    text = replace_once(text, "static void DecodeICS(", "static int DecodeICS(")
    text = replace_once(text, "\tglobalGain = GetBits(bsi, 8);", "\tif (psi->sampRateIdx < 0 || psi->sampRateIdx >= NUM_SAMPLE_RATES)return -1;\n\tglobalGain = GetBits(bsi, 8);")
    text = replace_once(text, "\tDecodeSectionData(bsi, icsInfo->winSequence, icsInfo->numWinGroup, icsInfo->maxSFB, psi->sfbCodeBook[ch]);", "\tif (!dbt_helix_ics_valid(icsInfo, psi->sampRateIdx) || dbt_helix_bits_left(bsi) < 0)return -1;\n\tif (DecodeSectionData(bsi, icsInfo->winSequence, icsInfo->numWinGroup, icsInfo->maxSFB, psi->sfbCodeBook[ch]) != 0)return -1;")
    text = replace_once(text, "\tif (pi->pulseDataPresent)\n\t\tDecodePulseInfo(bsi, pi);", "\tif (pi->pulseDataPresent) {\n\t\tDecodePulseInfo(bsi, pi);\n\t\tif (!dbt_helix_pulse_valid(icsInfo, pi, psi->sampRateIdx))return -1;\n\t}")
    text = replace_once(text, "\tif (ti->tnsDataPresent)\n\t\tDecodeTNSInfo(bsi, icsInfo->winSequence, ti, ti->coef);", "\tif (ti->tnsDataPresent && DecodeTNSInfo(bsi, icsInfo->winSequence, ti, ti->coef) != 0)return -1;")
    end = text.index("\n}\n", text.index("static int DecodeICS("))
    text = text[:end] + "\n\treturn dbt_helix_bits_left(bsi) < 0 ? -1 : 0;" + text[end:]
    text = replace_once(text, "\tDecodeICS(psi, &bsi, ch);", "\tif (DecodeICS(psi, &bsi, ch) != 0)return ERR_AAC_INVALID_FRAME;")
    text = replace_once(text, "\tbitsUsed = CalcBitsUsed(&bsi, *buf, *bitOffset);", "\tif (dbt_helix_bits_left(&bsi) < 0)return ERR_AAC_INDATA_UNDERFLOW;\n\tbitsUsed = CalcBitsUsed(&bsi, *buf, *bitOffset);\n\tif (bitsUsed < 0 || bitsUsed > *bitsAvail)return ERR_AAC_INDATA_UNDERFLOW;")
    text = replace_once(text, "\t/* decode next scalefactor from bitstream */", "\tif (dbt_helix_bits_left(bsi) <= 0)return 0;\n\t/* decode next scalefactor from bitstream */")
    text = text.replace("GetBits(", "dbt_helix_read_bits(").replace("AdvanceBitstream(", "dbt_helix_advance_bits(").replace("ByteAlignBitstream(", "dbt_helix_byte_align(")
    return text + "\nint dbt_helix_spectral_guard(void) { return 1; }\n"

def guard_elements(text):
    text = replace_once(text, '#include "coder.h"', '#include "coder.h"\n#include "helix_internal_guards.h"')
    needle = "\t\tDecodeICSInfo(bsi, icsInfo, psi->sampRateIdx);"
    text = replace_once(text, needle, "\t\tif (psi->sampRateIdx < 0 || psi->sampRateIdx >= NUM_SAMPLE_RATES)return -1;\n" + needle + "\n\t\tif (!dbt_helix_ics_valid(icsInfo, psi->sampRateIdx) || dbt_helix_bits_left(bsi) < 0)return -1;")
    text = replace_once(text, "\t\t\t\t\t\tmaskPtr++;\n\t\t\t\t\t\t*maskPtr = 0;", "\t\t\t\t\t\tmaskPtr++;\n\t\t\t\t\t\tif (maskPtr < psi->msMaskBits + sizeof(psi->msMaskBits))*maskPtr = 0;")
    text = replace_once(text, "\tbitsUsed = CalcBitsUsed(&bsi, *buf, *bitOffset);", "\tbitsUsed = CalcBitsUsed(&bsi, *buf, *bitOffset);\n\tif (bitsUsed < 0 || bitsUsed > *bitsAvail)return ERR_AAC_INDATA_UNDERFLOW;")
    text = replace_once(text, "\t/* update bitstream reader */", "\tif (dbt_helix_bits_left(&bsi) < 0)return ERR_AAC_INDATA_UNDERFLOW;\n\t/* update bitstream reader */")
    text = text.replace("GetBits(", "dbt_helix_read_bits(").replace("AdvanceBitstream(", "dbt_helix_advance_bits(").replace("ByteAlignBitstream(", "dbt_helix_byte_align(")
    return text + "\nint dbt_helix_element_guard(void) { return 1; }\n"

def guard_huffman(text):
    text = replace_once(text, '#include "coder.h"', '#include "coder.h"\n#include "helix_internal_guards.h"')
    needle = "\t\t\twhile (GetBits(bsi, 1) == 1)\n\t\t\t\tn++;"
    if text.count(needle) != 2:
        raise RuntimeError("Unexpected Helix escape sites")
    replacement = "\t\t\twhile (GetBits(bsi, 1) == 1) {\n\t\t\t\tif (++n > 30) {bsi->cachedBits=-(bsi->nBytes*8+1);return;}\n\t\t\t}"
    text = text.replace(needle, replacement)
    text = text.replace("GetBits(", "dbt_helix_read_bits(").replace("AdvanceBitstream(", "dbt_helix_advance_bits(")
    return text + "\nint dbt_helix_huffman_guard(void) { return 1; }\n"

def generate(source, output):
    if source.name != "aacdec.c":
        raise RuntimeError("Unreviewed Helix AAC source: " + source.name)
    sources = {}
    for name, expected in PINS.items():
        raw_source = (source.parent / name).read_bytes()
        raw = raw_source.replace(b"\r\n", b"\n")
        if hashlib.sha256(raw).hexdigest() != expected:
            raise RuntimeError("Unreviewed Helix AAC source: " + name)
        sources[name] = (raw_source, raw)
    text = sources["aacdec.c"][1].decode("utf-8")
    replacements = {
        '#include "aaccommon.h"':
            '#include "aaccommon.h"\n#ifdef AAC_ENABLE_SBR\n#error "Native AAC-LC requires SBR disabled"\n#endif\n'
            '_Static_assert(AAC_MAX_NCHANS == 2, "Native AAC PCM workspace requires two-channel bounds");',
        "if (baseChan + elementChans > AAC_MAX_NCHANS)":
            "if (baseChan + elementChans > AAC_MAX_NCHANS || baseChan + elementChans > aacDecInfo->nChans)",
        "\t/* byte align after each raw_data_block */":
            "\t/* Require exactly the configured channel layout before reporting success. */\n"
            "\tif (baseChan != aacDecInfo->nChans)\n\t\treturn ERR_AAC_NCHANS_TOO_HIGH;\n\n"
            "\t/* byte align after each raw_data_block */",
    }
    for before, after in replacements.items():
        text = replace_once(text, before, after)
    text += "\n/* Link-time proof that the native adapter uses the guarded unit. */\nint dbt_helix_channel_guard(void) { return 1; }\n"
    generated = {"aacdec_guarded.c": text,
                 "noiseless_guarded.c": guard_noiseless(sources["noiseless.c"][1].decode("utf-8")),
                 "decelmnt_guarded.c": guard_elements(sources["decelmnt.c"][1].decode("utf-8")),
                 "huffman_guarded.c": guard_huffman(sources["huffman.c"][1].decode("utf-8"))}
    output.mkdir(parents=True, exist_ok=True)
    hashes = {}
    for name, value in generated.items():
        data = value.encode("utf-8")
        if len(data) > 65536:
            raise RuntimeError("Unexpected generated Helix unit size: " + name)
        (output / name).write_bytes(data)
        hashes[name] = hashlib.sha256(data).hexdigest()
    report = {"source_sha256": PINS["aacdec.c"],
              "raw_source_sha256": hashlib.sha256(sources["aacdec.c"][0]).hexdigest(),
              "generated_sha256": hashes["aacdec_guarded.c"], "guard_sites": 2,
              "source_pins": PINS, "generated_units": hashes,
              "side_information_guards": ["ICS bands/groups", "positive bounded sections", "TNS order/coefficient capacity", "pulse table/positions", "common-window mask end", "sticky checked bit reads", "bounded positive-int spectral escapes"]}
    (output / "aac_port.json").write_text(json.dumps(report, indent=2) + "\n")
    return report

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(generate(args.source, args.output), indent=2))
