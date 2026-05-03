# BrainFPV firmware packer integration
#
# Wraps the post-link Intel HEX (already VMA-adjusted to 0x90400000) into the
# BrainFPV-format .bin that the bootloader accepts via drag-and-drop. The
# packer is vendored as a submodule under lib/main/brainfpv_fw_packer/ and
# is invoked directly as a Python script — no PATH-based wrapper required.
#
# One-time setup the user must do (Python deps for the packer):
#     pip install intelhex pyelftools pycrc
# Or, equivalently, install the packer itself in editable mode:
#     pip install -e lib/main/brainfpv_fw_packer/
#
# Usage after a successful build:
#     make CONFIG=RADIX2HD pack
# Produces:  obj/main/betaflight_<version>_RADIX2HD.brfp.bin
#
# Only included when BRAINFPV_BL=yes. Wired in from the main Makefile.

BRAINFPV_PACKER_DIR    := lib/main/brainfpv_fw_packer
BRAINFPV_PACKER_SCRIPT := $(BRAINFPV_PACKER_DIR)/brainfpv_fw_packer/brainfpv_fw_packer.py
TARGET_PACKED_BIN      := $(BIN_DIR)/$(TARGET_FULLNAME).brfp.bin

# Sanity check the submodule is initialized. If empty, prompt the user to
# init it rather than silently failing partway through the pack step.
$(BRAINFPV_PACKER_SCRIPT):
	@if [ ! -f "$@" ]; then \
	    echo "ERROR: brainfpv_fw_packer submodule not initialized."; \
	    echo "Run: git submodule update --init $(BRAINFPV_PACKER_DIR)"; \
	    exit 1; \
	fi

$(TARGET_PACKED_BIN): $(TARGET_HEX) $(BRAINFPV_PACKER_SCRIPT)
	@echo "Packing $(TARGET_HEX) for BrainFPV bootloader -> $(TARGET_PACKED_BIN)" "$(STDOUT)"
	$(V1) $(PYTHON) $(BRAINFPV_PACKER_SCRIPT) \
	    --name "Betaflight" \
	    --version $(FC_VER) \
	    --dev radix2hd \
	    --t firmware \
	    --boot $(EXST_ADJUST_VMA) \
	    --zip \
	    --in $< \
	    --out $@

.PHONY: pack
pack: $(TARGET_PACKED_BIN)
