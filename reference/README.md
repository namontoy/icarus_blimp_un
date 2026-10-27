# Reference documents

The primary sources the docs and code cite. They are third-party copyrighted
documents, so they are **not committed**. Download them into this folder; the
filenames below are the ones `.gitignore` expects and the docs refer to.

| Save as | Document | Source |
|---|---|---|
| `IAR_LARC2026_rulebook.pdf` | IAR-LARC 2026 Category Rules (dated 27 Mar 2026) | <https://mecatronica.umng.edu.co/index.html/rules/IAR_LARC2026.pdf> (also linked from <https://sites.google.com/oakland.edu/iar-larc/rules>) |
| `MochiSwarm_arXiv_2503.03077.pdf` | Xu, Vu, D'Antonio, Saldaña, *MochiSwarm: A testbed for robotic blimps in realistic environments* (ICRA 2025) | <https://arxiv.org/abs/2503.03077> |

`docs/05-race-strategy.md` and `sim/scene/build_scene.py` cite
`IAR_LARC2026_rulebook.txt`, a plain-text extract of the rulebook made so it
can be grepped. They cite it by rule number, so the PDF serves equally well.
If you want the text copy, any PDF-to-text tool works, e.g.

    pdftotext -layout IAR_LARC2026_rulebook.pdf IAR_LARC2026_rulebook.txt

If the organizers publish a newer rulebook revision, note its date here and
re-check the rule numbers quoted in the README and `docs/01-competition-brief.md`.
