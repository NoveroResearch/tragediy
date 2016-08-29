# tragediy - Track Generator DIY
Generate printable Tracks for Anki (Over)Drive vehicles
https://github.com/NoveroResearch/tragediy

### Disclaimer
This software is solely designed for creating and using tracks in combination with the [drive-sdk](https://github.com/anki/drive-sdk) provided by Anki. It is not intended to be used as a base for a commercial product competing with products sold by Anki.
The [Novero GmbH](http://novero.com) is in no way affiliated to Anki. All naming rights for Anki, Anki Drive and Anki Overdrive are property of [Anki](http://anki.com).

## About
### Intention

As an automotive company we at Novero are looking into different possibilities to try out traffic scenarios. Beside simulations we are trying out "real world" scenarios that aren't too expensive when something goes wrong. That's why we started to work with Anki vehicles. Some more details, including some videos can be found on our [CEOs Blog](http://freddiegeier.com/en/innovation-at-novero-why-we-are-excited-about-model-cars/).

As Anki hands out the drive-sdk, we decided to also give something back to the public.

### The code

This code can be used in two ways.

  1. It generates custom tracks which can be printed (svg-files).
  2. It allows to map readings from Anki vehicles back to x,y-coordinates. Useful for localization and visualization.

To generate printouts build this project and run its binary (See instructions below).

To handle readings you get from Anki vehicles driving on your printouts you can include the classes Track and LocationTable in your code. Basically these classes help you to translate location readings into x,y-coordinates.

### Usability of generated tracks
This code is intended to be used with the Anki drive-sdk. You won't be able to use it with the official Anki iOS/Android App as the app only supports Anki generated tracks. The exception is the "starter track" included with this code as it approximates the official Anki mat and so the app won't notice a difference.

So to find this code useful you need to build your own control program based on drive-sdk.

### Compatibility with vehicles
We got different results with different Anki firmware versions. The current Drive firmware is reading most of the encoded data (about 95%). The Overdrive firmware is much more picky, so the current version only reads about 70%.
If you print out a track and it's not working at all you might have a problem with your ink. If it is not visible in the infrared range the cars are basically seeing just white paper.

## Dependencies

### cmake
This is a cmake based project.

Installing cmake on OS X using homebrew:

	brew install cmake

Linux (debian):

	sudo apt-get install cmake

### libboost
The code depends on the boost library with the following modules: `program_options system filesystem`

On OS X the following command should be sufficient if you have homebrew installed:

	brew install boost --c++11

Linux (debian):
	
	sudo apt-get install libboost-dev libboost-filesystem-dev libboost-program-options-dev

## Build

The compiler is required to support C++14 language features. The code was tested to compile with

 * Apple's clang 7.2 on OS X 10.11.2,
 * gcc 5.3.1 on Ubuntu 16.04,
 * clang 3.6 on Ubuntu 16.04,
 * clang 3.8 on Ubuntu 16.04.

### Makefile

	git clone git@github.com:NoveroResearch/tragediy.git
	cd tragediy
	mkdir build
	cd build
	cmake ..
	make

### XCode

	git clone git@github.com:NoveroResearch/tragediy.git
	cd tragediy
	mkdir build
	cd build
	cmake -G Xcode ..
	open tragediy.xcodeproj

## Usage

In this version the tool can generate two types of tracks: starter and ring.
"starter" is a more or less exact reproduction of the track that comes with the Anki drive starter kit. "ring" is a more simple track that is 0-shaped. It fits on 4 DIN-A3 sheets of paper.

To generate a ring track which fits on 4 sheets of paper use the following command:

	./tragediy -t ring -s a3-landscape

To import the crossroads track from Anki Drive app data use the following command:

	./tragediy -I $SOME_DIRECTORY/com.anki.drive/ -i IntersecProduction_map.txt --prefix crossroads

To import one of the predefined tracks from Anki Overdrive app data use the following command:

	./tragediy -I $SOME_DIRECTORY/com.anki.overdrive/ -j modular_micro.txt --prefix micro

### Output:

 * `${prefix}_track_clean.svg` this file is the full sized track. If you own a big enough printer you could just print this.
 * `${prefix}_track_annotated.svg` basically the same file as above but it shows at which point readings are expected.
 * `${prefix}_location-table.*` these files allow to programmatically map vehicle readings to coordinates.
 * `${prefix}_track_XxY.svg` these files are the tiles you could use for printing on a normal printer and glue together afterwards. How many of those files you get depends on the selected paper size.

### Adding of tracks
Currently the available tracks are hardcoded. Take a look at `constructRingTrack` and `constructStarterTrack` in Track.cpp. From this you should get an idea how to define a new track. Same applies for papersizes.


