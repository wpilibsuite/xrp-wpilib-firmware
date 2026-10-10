"""Build a project-local BTstack L2CAP override with legal LE channel IDs.

Arduino-Pico ships L2CAP in a prebuilt archive. Compile the matching bundled
source with this small allocator correction so the fix survives clean builds
without modifying the shared PlatformIO package. Also imported by native tests.
"""

from pathlib import Path


def patch_l2cap(source):
    replacements = [
        (
            "static uint16_t l2cap_next_local_cid(void);",
            "static uint16_t l2cap_next_local_cid(bd_addr_type_t address_type);",
        ),
        (
            """static uint16_t l2cap_next_local_cid(void){
    do {
        if (l2cap_local_source_cid == 0xfffeu) {""",
            """static uint16_t l2cap_next_local_cid(bd_addr_type_t address_type){
    // LE dynamic CIDs are 0x0040..0x007f; Classic retains its wider range.
    // The shared counter can already exceed the LE range after Classic use.
    const uint16_t max_cid = address_type == BD_ADDR_TYPE_ACL ? 0xfffeu : 0x007fu;
    do {
        if (l2cap_local_source_cid >= max_cid) {""",
        ),
        (
            "channel->local_cid = l2cap_next_local_cid();",
            "channel->local_cid = l2cap_next_local_cid(address_type);",
        ),
    ]
    for original, replacement in replacements:
        if source.count(original) != 1:
            raise RuntimeError(
                "Bundled BTstack L2CAP source changed; review the LE CID patch "
                "in tools/btstack_l2cap.py before building."
            )
        source = source.replace(original, replacement, 1)
    return source


def configure_build(env):
    framework = Path(env.PioPlatform().get_package_dir("framework-arduinopico"))
    source = framework / "pico-sdk/lib/btstack/src/l2cap.c"
    build_dir = Path(env.subst("$BUILD_DIR")) / "btstack_l2cap"
    generated = build_dir / "src/l2cap.c"
    patched = patch_l2cap(source.read_text())
    generated.parent.mkdir(parents=True, exist_ok=True)
    if not generated.exists() or generated.read_text() != patched:
        generated.write_text(patched)
    # Resolve L2CAP from our override before the framework's prebuilt archive.
    # Use the framework's final flags/configuration, including Classic + LE.
    library = env.BuildLibrary(str(build_dir / "override"), str(generated.parent))
    env.Prepend(LIBS=[library])


# PlatformIO executes this as a post extra_script, after framework setup.
if "Import" in globals():
    Import("env")
    configure_build(env)
