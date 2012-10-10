sudo su
echo "export LIBOVERLAY_SCROLLBAR=0" > /etc/X11/Xsession.d/80overlayscrollbars
sudo apt-get remove overlay-scrollbar liboverlay-scrollbar-0.1-0
sudo apt-get install gconf-editor
