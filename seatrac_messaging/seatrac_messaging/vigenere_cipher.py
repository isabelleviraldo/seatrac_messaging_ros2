#!/usr/bin/env python2.7

"""
vigenere_cipher.py

Provides functions to encrypt and decrypt text using a Vigenere cipher
applied over the printable ASCII range (codes 32 to 126).
"""

# Build the universe of allowed characters
UNIVERSE = [chr(i) for i in range(32, 127)]
UNI_LEN = len(UNIVERSE)


def _vigenere(text, key, mode):
    """
    Core Vigenere cipher routine.

    :param text: The input text to encrypt or decrypt.
    :param key: The cipher key (must consist of ASCII chars 32 to 126).
    :param mode: 'e' to encrypt, 'd' to decrypt.
    :return: The resulting cipher text or plain text.

    Raises ValueError on invalid inputs.
    """
    if not text:
        raise ValueError("Text must not be empty.")
    if not key:
        raise ValueError("Key must not be empty.")
    if mode not in ('e', 'd'):
        raise ValueError("Mode must be 'e' (encrypt) or 'd' (decrypt).")
    if any(k not in UNIVERSE for k in key):
        raise ValueError(
            "Key contains invalid characters. Only ASCII 32 to 126 are allowed."
        )

    result = []
    k_len = len(key)

    for i, ch in enumerate(text):
        if ch not in UNIVERSE:
            # Leave characters outside the universe unchanged
            result.append(ch)
        else:
            txt_idx = UNIVERSE.index(ch)
            key_idx = UNIVERSE.index(key[i % k_len])
            if mode == 'd':
                key_idx = -key_idx
            result.append(UNIVERSE[(txt_idx + key_idx) % UNI_LEN])

    return ''.join(result)


def v_encrypt(text, key):
    """
    Encrypt the given text with the provided key.

    :param text: Plain text to encrypt.
    :param key: Cipher key.
    :return: Encrypted text.
    """
    return _vigenere(text, key, 'e')


def v_decrypt(text, key):
    """
    Decrypt the given text with the provided key.

    :param text: Cipher text to decrypt.
    :param key: Cipher key.
    :return: Decrypted plain text.
    """
    return _vigenere(text, key, 'd')
