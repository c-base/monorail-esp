#!/usr/bin/env bun
import * as BMP from "bmp-js";
import * as fs from "fs";
import * as path from "path";

const PROGRAM_NAME = "bmp-to-xbm-c.mjs";

function main() {
  const argv = process.argv.slice(2);
  const [filePathRaw, varName = "img"] = argv;

  if (!filePathRaw) {
    throw new Error(`usage: ${PROGRAM_NAME} <filename> [var_name]`);
  }
  const filePath = path.resolve(filePathRaw);
  const bmpBytes = fs.readFileSync(filePath);

  const xbmImage = bmpToXBM(bmpBytes);
  const cCode = bytesToCCode(xbmImage, varName, filePath);
  console.log(cCode);
}

/** @typedef {{ width: number, height: number, data: Buffer }} XBMImage */

/** @returns {XBMImage} */
function bmpToXBM(/** @type {Buffer} */ bmpData) {
  const bmp = BMP.decode(bmpData);

  const bytes = [];
  let indexInRow = 0;
  let currentByte = 0b00000000;
  let bitOffset = 0;
  for (let i = 0; i < bmp.data.length; i += 4) {
    const pixel = bmp.data.readUInt32LE(i);
    const isPixelOn = pixel !== 0;

    if (isPixelOn) {
      // the lowest bit is the leftmost pixel
      // . . . . . . . .
      // 0 1 2 3 4 5 6 7
      currentByte |= 1 << bitOffset;
    }
    if (
      // completed a byte
      bitOffset === 7 ||
      // next pixel starts a full row, need to finish now
      (bitOffset < 7 && (indexInRow + 1) % bmp.width === 0)
    ) {
      bytes.push(currentByte);
      currentByte = 0b00000000;
      bitOffset = 0;
    } else {
      bitOffset++;
    }
    indexInRow = (indexInRow + 1) % bmp.width;
  }

  return { width: bmp.width, height: bmp.height, data: bytes };
}

const BYTES_PER_ROW = 19;
const INDENT = "    ";

/** @returns {string} */
function bytesToCCode(
  /** @type {XBMImage} */ xbmImage,
  /** @type {string} */ varName,
  /** @type {string} */ sourceFilePath
) {
  const { width, height, data } = xbmImage;
  let res = [];
  res.push(
    `#include <U8g2lib.h>`,
    ``,
    `// generated from '${path.basename(
      sourceFilePath
    )}' using \`${PROGRAM_NAME}\``
  );

  const bitsVarName = `__embedded_xbm_bits_${varName}`;
  // note: the U8X8_PROGMEM is important, we don't want the imge to be in RAM
  res.push(
    `static const unsigned char ${bitsVarName}[${data.length}] U8X8_PROGMEM = {`
  );
  {
    const pushRow = (row) => {
      res.push(
        INDENT +
          row
            .map((byte) => {
              return `0x${byte.toString(16).padStart(2, "0").toUpperCase()}`;
            })
            .join(", ") +
          ","
      );
    };
    let row = [];
    for (const byte of data) {
      row.push(byte);
      if (row.length === BYTES_PER_ROW) {
        pushRow(row);
        row = [];
      }
    }
    if (row.length > 0) {
      pushRow(row);
    }
  }

  res.push(`};`, ``);

  // use an anonymous struct because importing a declaration would need a relative path so it'd be hard
  res.push(
    `struct {`,
    `${INDENT}u8g2_uint_t width;`,
    `${INDENT}u8g2_uint_t height;`,
    `${INDENT}const uint8_t *data;`,
    `} ${varName} = {.width = ${width}, .height = ${height}, .data = (uint8_t *)${bitsVarName}};`
  );

  return res.join("\n") + "\n";
}

main();
