# stick_verify.ko (STMicroelectronics STICK ST1Wire + stickapi + libstickcrypto)
obj-m += stick_verify.o

STICK_CFLAGS := \
    -DLTC_NOTHING \
    -DLTC_NO_FILE \
    -DLTC_RIJNDAEL \
    -DLTC_SHA512 \
    -DLTC_CCM_MODE \
    -DLTC_NO_TEST \
    -DLTC_CURVE25519 \
    -DLTC_DER \
    -DLTC_MPI \
    -DLTC_OMAC \
    -DLTC_HASH_HELPERS \
    -DLTC_STICK \
    -DLTC_SOURCE \
    -DLTC_NO_WCHAR \
    -DRELEASE_BUILDING=1 \
    -fvisibility=hidden \
    -DARGTYPE=1 \
    -DRELEASE_CUSTID=00 \
    -DRELEASE_NAME=dev \
    -DRELEASE_TYPE=DEV -DSTICK_DEBUG_LOG=1

stick_verify-objs := \
    stick.o \
    stickapi/stickapitest.o \
    stickapi/stick_api.o \
    stickapi/libsticknvm.o \
    stickapi/st1wire_kernel.o \
    stickapi/stick_core.o \
    stickapi/stick_kernel_platform.o \
    stickapi/stick_NVM_attributes.o \
    stickapi/libstickcrypto/libstickcrypto.o \
    stickapi/libstickcrypto/libstickcrypto_rng.o \
    stickapi/libstickcrypto/aes.o  \
    stickapi/libstickcrypto/aes_tab.o  \
    stickapi/libstickcrypto/ccm_add_aad.o  \
    stickapi/libstickcrypto/ccm_add_nonce.o  \
    stickapi/libstickcrypto/ccm_done.o  \
    stickapi/libstickcrypto/ccm_init.o  \
    stickapi/libstickcrypto/ccm_memory.o  \
    stickapi/libstickcrypto/ccm_process.o  \
    stickapi/libstickcrypto/ccm_reset.o  \
    stickapi/libstickcrypto/hash_memory.o  \
    stickapi/libstickcrypto/sha512.o  \
    stickapi/libstickcrypto/omac_done.o  \
    stickapi/libstickcrypto/omac_init.o  \
    stickapi/libstickcrypto/omac_memory.o  \
    stickapi/libstickcrypto/omac_memory_multi.o  \
    stickapi/libstickcrypto/omac_process.o  \
    stickapi/libstickcrypto/copy_or_zeromem.o  \
    stickapi/libstickcrypto/crypt_argchk.o  \
    stickapi/libstickcrypto/crypt_cipher_descriptor.o  \
    stickapi/libstickcrypto/crypt_cipher_is_valid.o  \
    stickapi/libstickcrypto/crypt_find_cipher.o  \
    stickapi/libstickcrypto/crypt_find_hash.o  \
    stickapi/libstickcrypto/crypt_hash_descriptor.o  \
    stickapi/libstickcrypto/crypt_hash_is_valid.o  \
    stickapi/libstickcrypto/crypt_prng_descriptor.o  \
    stickapi/libstickcrypto/crypt_prng_is_valid.o  \
    stickapi/libstickcrypto/crypt_register_cipher.o  \
    stickapi/libstickcrypto/crypt_register_hash.o  \
    stickapi/libstickcrypto/error_to_string.o  \
    stickapi/libstickcrypto/mem_neq.o  \
    stickapi/libstickcrypto/zeromem.o  \
    stickapi/libstickcrypto/ec25519_export.o  \
    stickapi/libstickcrypto/tweetnacl.o  \
    stickapi/libstickcrypto/ed25519_export.o  \
    stickapi/libstickcrypto/ed25519_import_raw.o  \
    stickapi/libstickcrypto/ed25519_make_key.o  \
    stickapi/libstickcrypto/ed25519_sign.o  \
    stickapi/libstickcrypto/ed25519_verify.o  \
    stickapi/libstickcrypto/x25519_export.o  \
    stickapi/libstickcrypto/x25519_import_raw.o  \
    stickapi/libstickcrypto/x25519_make_key.o  \
    stickapi/libstickcrypto/x25519_shared_secret.o  \
    stickapi/libstickcrypto/stick_kzalloc.o

# per-object flags (only stick_verify objects get the LTC/STICK defines)
$(foreach o,$(stick_verify-objs),$(eval CFLAGS_$(o) += $(STICK_CFLAGS)))
# stock __FILE__ strings (LTC_ARGCHK / stick asserts)
$(foreach o,$(stick_verify-objs),$(eval CFLAGS_$(o) += -fmacro-prefix-map=$(src)/=drivers/power/supply/battery_secret/))
