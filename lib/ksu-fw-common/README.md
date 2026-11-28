# ksu-fw-common (vendored)

Copied from [KSU-MS/ksu-fw-common](https://github.com/KSU-MS/ksu-fw-common)
at commit `9100f855745889a15ca1d7e19beeaf58632d9f1d`, which is what
`lib_deps` resolved to. MIT licensed (`library.json`).

Vendored so the CAN receive fix (interrupt-driven receive into a queue
instead of polling FlexCAN_T4's 4 standard-ID mailboxes) can be reviewed
and tested here before going upstream. The changes since that commit are
their own commits in this repo's history:

    git log --oneline -- lib/ksu-fw-common

Once upstream has them, delete this directory and put the
`https://github.com/KSU-MS/ksu-fw-common` line back in `platformio.ini`.
