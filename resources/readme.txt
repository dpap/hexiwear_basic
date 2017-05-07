Adding an image
https://www.hackster.io/keithmitchell/drawing-images-on-the-hexiwear-oled-3c9517
steps: 
Gimp -> image must be 96x96
	export as bmp
	Compatibility Settings” check “Do not write color space information”,
	Advanced Options”, change it to “16-bit: R5 G5 B5” (A1 R5 G5 B5 or X1 R5 G5 B5)
	Tweak color settings: colors: Hue and saturation
HEXIWEAR-master\SW\ResourceCollectionTool -Add image -> generate
Open "newname.c + newname.h" 
copy to Eclipse 
	-> include EmbeddedTypes.h to get "uint8_t" type
	-> <>.h ->copy newname.h contents =>remove "code" and replace with uint8_t extern const uint8_t agile96_bmp[18438];
	-> <>.c -> copy newname.c contents
