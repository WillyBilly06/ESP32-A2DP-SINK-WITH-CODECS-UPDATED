"""Generate one guarded Helix unit, retaining its original license notice."""
import argparse
import hashlib
import json
from pathlib import Path


def generate(source, output):
    raw = source.read_bytes()
    if hashlib.sha256(raw).hexdigest() != "f3260aeca52c91675761b8dccf2ec83cf42f8cc07ef47da10676ec1aa66ea5d6":
        raise RuntimeError("Unreviewed Helix AAC source; audit the guard before updating its hash")
    text = raw.decode("utf-8")
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
        if text.count(before) != 1:
            raise RuntimeError("Unexpected Helix AAC source: guard site changed")
        text = text.replace(before, after)
    text += "\n/* Link-time proof that the native adapter uses the guarded unit. */\nint dbt_helix_channel_guard(void) { return 1; }\n"
    data = text.encode("utf-8")
    if len(data) > 65536:
        raise RuntimeError("Unexpected generated Helix unit size")
    output.mkdir(parents=True, exist_ok=True)
    (output / "aacdec_guarded.c").write_bytes(data)
    report = {"source_sha256": hashlib.sha256(raw).hexdigest(),
              "generated_sha256": hashlib.sha256(data).hexdigest(),
              "guard_sites": 2}
    (output / "aac_port.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(generate(args.source, args.output), indent=2))
