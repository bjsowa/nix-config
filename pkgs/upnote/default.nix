{ appimageTools, fetchurl }:

appimageTools.wrapType2 {
  pname = "upnote";
  version = "9.10.4";

  src = fetchurl {
    url = "https://download.getupnote.com/app/UpNote.AppImage";
    hash = "";
  };
}
