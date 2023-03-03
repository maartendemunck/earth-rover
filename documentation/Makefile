documentation.pdf: src/documentation.lyx
	lyx --export-to pdf5 $@ $^

documentation-book.pdf: documentation.pdf
	pdfbook2 --paper=a4paper --no-crop --outer-margin=0 --inner-margin=0 --top-margin=0 --bottom-margin=0 $^
