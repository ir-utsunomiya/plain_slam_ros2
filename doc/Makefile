TARGET = root
FINAL_TARGET = paper
NKF = -u

all:
	nkf $(NKF) --overwrite *.tex
	platex $(TARGET).tex
	bibtex $(TARGET).aux
	platex $(TARGET).tex

pdf:
	nkf $(NKF) --overwrite *.tex
	platex $(TARGET).tex
	bibtex $(TARGET).aux
	platex $(TARGET).tex
	xdvipdfmx $(TARGET).dvi

pdfopt:
	nkf $(NKFOPTION) --overwrite ./*.tex
	# nkf $(NKFOPTION) --overwrite ./*.sty
	nkf $(NKFOPTION) --overwrite ./*.cls
	platex $(TARGET).tex
	bibtex $(TARGET).aux
	platex $(TARGET).tex
#	dvipdfmx -f ptex-ipa.map $(TARGET).dvi
	dvipdfmx -p letter -f ptex-ipa.map $(TARGET).dvi
	gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/default -dNOPAUSE -dQUIET -dBATCH -sOutputFile=$(FINAL_TARGET)_opt.pdf $(TARGET).pdf
	gs -q -dNOPAUSE -dBATCH -dPDFSETTINGS=/prepress -sDEVICE=pdfwrite -sOutputFile=$(FINAL_TARGET).pdf $(FINAL_TARGET)_opt.pdf

clean:
	rm -rf $(TARGET).log
	rm -rf $(TARGET).aux
	rm -rf $(TARGET).dvi
	rm -rf $(TARGET).pdf
	rm -rf $(FINAL_TARGET)_opt.pdf
	rm -rf $(FINAL_TARGET).pdf
	rm -rf $(TARGET).bbl
	rm -rf $(TARGET).blg
	rm -rf missfont.log
