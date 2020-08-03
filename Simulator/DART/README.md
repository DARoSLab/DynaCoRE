Comparable to DART: release-6.4 branch

You can download it in Dyna-Core/dart and use dynacore-dart branch

## Recording
In dart, we could set a recorder as below.
viewer.record("PATH_TO_SAVE");
Then png files are saved at each frame.

In order to generate a video from it, the best option seems to be FFMPEG.
So install FFMPEG for your OS.
For mac os, use homebrew to install as below.

--------
brew install ffmpeg --with-fdk-aac --with-ffplay --with-freetype --with-frei0r --with-libass --with-libvo-aacenc --with-libvorbis --with-libvpx --with-opencore-amr --with-openjpeg --with-opus --with-rtmpdump --with-schroedinger --with-speex --with-theora --with-tools
--------

Then in the directory where the pngs are saved, hit the command, for example,

-------
ffmpeg -i image%06d.png video.avi
-------

There is a nice manual described in https://en.wikibooks.org/wiki/FFMPEG_An_Intermediate_Guide/image_sequence
UnlikeYou like this