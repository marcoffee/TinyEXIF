/*
  TinyEXIF.h -- A simple ISO C++ library to parse basic EXIF and XMP
                information from a JPEG file.

  Copyright (c) 2015-2017 Seacave
  cdc.seacave@gmail.com
  All rights reserved.

  Based on the easyexif library (2013 version)
    https://github.com/mayanklahiri/easyexif
  of Mayank Lahiri (mlahiri@gmail.com).

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

   - Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY EXPRESS
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
  NO EVENT SHALL THE FREEBSD PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
  OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __TINYEXIF_H__
#define __TINYEXIF_H__

#include <string>
#include <vector>
#include <optional>

#define TINYEXIF_MAJOR_VERSION 1
#define TINYEXIF_MINOR_VERSION 0
#define TINYEXIF_PATCH_VERSION 1

#ifdef _MSC_VER
#   ifdef TINYEXIF_EXPORT
#       define TINYEXIF_LIB __declspec(dllexport)
#   elif defined(TINYEXIF_IMPORT)
#       define TINYEXIF_LIB __declspec(dllimport)
#   else
#       define TINYEXIF_LIB
#   endif
#elif __GNUC__ >= 4
#   define TINYEXIF_LIB __attribute__((visibility("default")))
#else
#   define TINYEXIF_LIB
#endif

namespace TinyEXIF {

enum ErrorCode {
	PARSE_SUCCESS           = 0, // Parse EXIF and/or XMP was successful
	PARSE_INVALID_JPEG      = 1, // No JPEG markers found in buffer, possibly invalid JPEG file
	PARSE_UNKNOWN_BYTEALIGN = 2, // Byte alignment specified in EXIF file was unknown (neither Motorola nor Intel)
	PARSE_ABSENT_DATA       = 3, // No EXIF and/or XMP data found in JPEG file
	PARSE_CORRUPT_DATA      = 4, // EXIF and/or XMP header was found, but data was corrupted
};

enum FieldCode {
	FIELD_NA                 = 0, // No EXIF or XMP data
	FIELD_EXIF               = (1 << 0), // EXIF data available
	FIELD_XMP                = (1 << 1), // XMP data available
	FIELD_ALL                = FIELD_EXIF|FIELD_XMP
};

class EntryParser;

//
// Interface class responsible for fetching stream data to be parsed
//
class TINYEXIF_LIB EXIFStream {
public:
	virtual ~EXIFStream() {}

	// Check the state of the stream.
	virtual bool IsValid() const = 0;

	// Return the pointer to the beginning of the desired size buffer
	// following current buffer position.
	virtual const uint8_t* GetBuffer(unsigned desiredLength) = 0;

	// Advance current buffer position with the desired size;
	// return false if stream ends in less than the desired size.
	virtual bool SkipBuffer(unsigned desiredLength) = 0;
};

//
// Class responsible for storing and parsing EXIF & XMP metadata from a JPEG stream
//
class TINYEXIF_LIB EXIFInfo {
public:
	EXIFInfo();
	EXIFInfo(EXIFStream& stream);
	EXIFInfo(std::istream& stream); // NB: the stream must have been opened in binary mode
	EXIFInfo(const uint8_t* data, unsigned length);

	// Parsing function for an entire JPEG image stream.
	//
	// PARAM 'stream': Interface to fetch JPEG image stream.
	// PARAM 'data': A pointer to a JPEG image.
	// PARAM 'length': The length of the JPEG image.
	// RETURN:  PARSE_SUCCESS (0) on success with 'result' filled out
	//          error code otherwise, as defined by the PARSE_* macros
	int parseFrom(EXIFStream& stream);
	int parseFrom(std::istream& stream); // NB: the stream must have been opened in binary mode
	int parseFrom(const uint8_t* data, unsigned length);

	// Parsing function for an EXIF segment. This is used internally by parseFrom()
	// but can be called for special cases where only the EXIF section is
	// available (i.e., a blob starting with the bytes "Exif\0\0").
	int parseFromEXIFSegment(const uint8_t* buf, unsigned len);

#ifndef TINYEXIF_NO_XMP_SUPPORT
	// Parsing function for an XMP segment. This is used internally by parseFrom()
	// but can be called for special cases where only the XMP section is
	// available (i.e., a blob starting with the bytes "http://ns.adobe.com/xap/1.0/\0").
	int parseFromXMPSegment(const uint8_t* buf, unsigned len);
	int parseFromXMPSegmentXML(const char* szXML, unsigned len);
#endif // TINYEXIF_NO_XMP_SUPPORT

	// Set all data members to default values.
	// Should be called before parsing a new stream.
	void clear();

private:
	// Parse tag as Image IFD.
	void parseIFDImage(EntryParser&, unsigned&, unsigned&);
	// Parse tag as Exif IFD.
	void parseIFDExif(EntryParser&);
	// Parse tag as GPS IFD.
	void parseIFDGPS(EntryParser&);
	// Parse tag as MakerNote IFD.
	void parseIFDMakerNote(EntryParser&);

public:
	// Data fields
	uint32_t Fields;                    // Store if EXIF and/or XMP data fields are available
	std::optional<uint32_t> ImageWidth;                // Image width reported in EXIF data
	std::optional<uint32_t> ImageHeight;               // Image height reported in EXIF data
	std::optional<uint32_t> RelatedImageWidth;         // Original image width reported in EXIF data
	std::optional<uint32_t> RelatedImageHeight;        // Original image height reported in EXIF data
	std::optional<std::string> ImageDescription;       // Image description
	std::optional<std::string> Make;                   // Camera manufacturer's name
	std::optional<std::string> Model;                  // Camera model
	std::optional<std::string> SerialNumber;           // Serial number of the body of the camera
	std::optional<uint16_t> Orientation;               // Image orientation, start of data corresponds to
									    // 0: unspecified in EXIF data
									    // 1: upper left of image
									    // 3: lower right of image
									    // 6: upper right of image
									    // 8: lower left of image
									    // 9: undefined
	std::optional<double> XResolution;                 // Number of pixels per ResolutionUnit in the ImageWidth direction
	std::optional<double> YResolution;                 // Number of pixels per ResolutionUnit in the ImageLength direction
	std::optional<uint16_t> ResolutionUnit;            // Unit of measurement for XResolution and YResolution
									    // 1: no absolute unit of measurement. Used for images that may have a non-square aspect ratio, but no meaningful absolute dimensions
									    // 2: inch
									    // 3: centimeter
	std::optional<uint16_t> BitsPerSample;             // Number of bits per component
	std::optional<std::string> Software;               // Software used
	std::optional<std::string> DateTime;               // File change date and time
	std::optional<std::string> DateTimeOriginal;       // Original file date and time (may not exist)
	std::optional<std::string> DateTimeDigitized;      // Digitization date and time (may not exist)
	std::optional<std::string> SubSecTimeOriginal;     // Sub-second time that original picture was taken
	std::optional<std::string> Copyright;              // File copyright information
	std::optional<double> ExposureTime;                // Exposure time in seconds
	std::optional<double> FNumber;                     // F/stop
	std::optional<uint16_t> ExposureProgram;           // Exposure program
	                                    // 0: not defined
	                                    // 1: manual
	                                    // 2: normal program
	                                    // 3: aperture priority
	                                    // 4: shutter priority
	                                    // 5: creative program
	                                    // 6: action program
	                                    // 7: portrait mode
	                                    // 8: landscape mode
	std::optional<uint16_t> ISOSpeedRatings;           // ISO speed
	std::optional<double> ShutterSpeedValue;           // Shutter speed (reciprocal of exposure time)
	std::optional<double> ApertureValue;               // The lens aperture
	std::optional<double> BrightnessValue;             // The value of brightness
	std::optional<double> ExposureBiasValue;           // Exposure bias value in EV
	std::optional<double> SubjectDistance;             // Distance to focus point in meters
	std::optional<double> FocalLength;                 // Focal length of lens in millimeters
	std::optional<uint16_t> Flash;                     // Flash info
	                                    // Flash used (Flash&1)
	                                    // 0: no flash, >0: flash used
	                                    // Flash returned light status ((Flash & 6) >> 1)
	                                    // 0: no strobe return detection function
	                                    // 1: reserved
	                                    // 2: strobe return light not detected
	                                    // 3: strobe return light detected
	                                    // Flash mode ((Flash & 24) >> 3)
	                                    // 0: unknown
	                                    // 1: compulsory flash firing
	                                    // 2: compulsory flash suppression
	                                    // 3: auto mode
	                                    // Flash function ((Flash & 32) >> 5)
	                                    // 0: flash function present, >0: no flash function
	                                    // Flash red-eye ((Flash & 64) >> 6)
	                                    // 0: no red-eye reduction mode or unknown, >0: red-eye reduction supported
	std::optional<uint16_t> MeteringMode;              // Metering mode
	                                    // 0: unknown
	                                    // 1: average
	                                    // 2: center weighted average
	                                    // 3: spot
	                                    // 4: multi-spot
	                                    // 5: pattern
	                                    // 6: partial
	std::optional<uint16_t> LightSource;               // Kind of light source
	                                    // 0: unknown
	                                    // 1: daylight
	                                    // 2: fluorescent
	                                    // 3: tungsten (incandescent light)
	                                    // 4: flash
	                                    // 9: fine weather
	                                    // 10: cloudy weather
	                                    // 11: shade
	                                    // 12: daylight fluorescent (D 5700 - 7100K)
	                                    // 13: day white fluorescent (N 4600 - 5400K)
	                                    // 14: cool white fluorescent (W 3900 - 4500K)
	                                    // 15: white fluorescent (WW 3200 - 3700K)
	                                    // 17: standard light A
	                                    // 18: standard light B
	                                    // 19: standard light C
										// 20: D55
										// 21: D65
										// 22: D75
										// 23: D50
	                                    // 24: ISO studio tungsten
	std::optional<uint16_t> ProjectionType;            // Projection type
									    // 0: unknown projection
									    // 1: perspective projection
									    // 2: equirectangular/spherical projection
	std::optional<std::vector<uint16_t>> SubjectArea;  // Location and area of the main subject in the overall scene expressed in relation to the upper left as origin, prior to rotation
	                                    // 0: unknown
	                                    // 2: location of the main subject as coordinates (first value is the X coordinate and second is the Y coordinate)
	                                    // 3: area of the main subject as a circle (first value is the center X coordinate, second is the center Y coordinate, and third is the diameter)
	                                    // 4: area of the main subject as a rectangle (first value is the center X coordinate, second is the center Y coordinate, third is the width of the area, and fourth is the height of the area)
	struct TINYEXIF_LIB Calibration_t { // Camera calibration information
		std::optional<double> FocalLength;             // Focal length (pixels)
		std::optional<double> OpticalCenterX;          // Principal point X (pixels)
		std::optional<double> OpticalCenterY;          // Principal point Y (pixels)
	} Calibration;
	struct TINYEXIF_LIB LensInfo_t {    // Lens information
		std::optional<double> FStopMin;                // Min aperture (f-stop)
		std::optional<double> FStopMax;                // Max aperture (f-stop)
		std::optional<double> FocalLengthMin;          // Min focal length (mm)
		std::optional<double> FocalLengthMax;          // Max focal length (mm)
		std::optional<double> DigitalZoomRatio;        // Digital zoom ratio when the image was shot
		std::optional<double> FocalLengthIn35mm;       // Focal length in 35mm film
		std::optional<double> FocalPlaneXResolution;   // Number of pixels in the image width (X) direction per FocalPlaneResolutionUnit on the camera focal plane (may not exist)
		std::optional<double> FocalPlaneYResolution;   // Number of pixels in the image width (Y) direction per FocalPlaneResolutionUnit on the camera focal plane (may not exist)
		std::optional<uint16_t> FocalPlaneResolutionUnit;// Unit for measuring FocalPlaneXResolution and FocalPlaneYResolution (may not exist)
										// 0: unspecified in EXIF data
										// 1: no absolute unit of measurement
										// 2: inch
										// 3: centimeter
		std::optional<std::string> Make;               // Lens manufacturer
		std::optional<std::string> Model;              // Lens model
	} LensInfo;
	struct TINYEXIF_LIB Geolocation_t { // GPS information embedded in file
		std::optional<double> Latitude;                // Image latitude expressed as decimal
		std::optional<double> Longitude;               // Image longitude expressed as decimal
		std::optional<double> Altitude;                // Altitude in meters, relative to sea level
		std::optional<int8_t> AltitudeRef;             // 0: above sea level, -1: below sea level
		std::optional<double> RelativeAltitude;        // Relative altitude in meters
		std::optional<double> RollDegree;              // Flight roll in degrees
		std::optional<double> PitchDegree;             // Flight pitch in degrees
		std::optional<double> YawDegree;               // Flight yaw in degrees
		std::optional<double> SpeedX;                  // Flight speed on X in meters/second
		std::optional<double> SpeedY;                  // Flight speed on Y in meters/second
		std::optional<double> SpeedZ;                  // Flight speed on Z in meters/second
		std::optional<double> AccuracyXY;              // GPS accuracy on XY in meters
		std::optional<double> AccuracyZ;               // GPS accuracy on Z in meters
		std::optional<double> GPSDOP;                  // GPS DOP (data degree of precision)
		std::optional<uint16_t> GPSDifferential;       // Differential correction applied to the GPS receiver (may not exist)
										// 0: measurement without differential correction
										// 1: differential correction applied
		std::optional<std::string> GPSMapDatum;        // Geodetic survey data (may not exist)
		std::optional<std::string> GPSTimeStamp;       // Time as UTC (Coordinated Universal Time) (may not exist)
		std::optional<std::string> GPSDateStamp;       // A character string recording date and time information relative to UTC (Coordinated Universal Time) YYYY:MM:DD (may not exist)
		struct Coord_t {
			std::optional<double> degrees;
			std::optional<double> minutes;
			std::optional<double> seconds;
			std::optional<uint8_t> direction;
		} LatComponents, LonComponents; // Latitude/Longitude expressed in deg/min/sec
		void parseCoords();             // Convert Latitude/Longitude from deg/min/sec to decimal
		bool hasLatLon() const;         // Return true if (lat,lon) is available
		bool hasAltitude() const;       // Return true if (alt) is available
		bool hasRelativeAltitude()const;// Return true if (rel_alt) is available
		bool hasOrientation() const;    // Return true if (roll,yaw,pitch) is available
		bool hasSpeed() const;          // Return true if (speedX,speedY,speedZ) is available
	} GeoLocation;
	struct TINYEXIF_LIB GPano_t {           // Spherical metadata. https://developers.google.com/streetview/spherical-metadata
		std::optional<double> PosePitchDegrees;        // Pitch, measured in degrees above the horizon, for the center in the image. Value must be >= -90 and <= 90.
		std::optional<double> PoseRollDegrees;         // Roll, measured in degrees, of the image where level with the horizon is 0. As roll increases, the horizon rotates counterclockwise in the image. Value must be > -180 and <= 180.
		bool hasPosePitchDegrees() const; // Return true if PosePitchDegrees is available
		bool hasPoseRollDegrees() const; // Return true if PoseRollDegrees is available
	} GPano;
	struct TINYEXIF_LIB MicroVideo_t {      // Google camera video file in metadata
		std::optional<uint32_t> HasMicroVideo;         // not zero if exists
		std::optional<uint32_t> MicroVideoVersion;     // just regularinfo
		std::optional<uint32_t> MicroVideoOffset;      // offset from end of file
	} MicroVideo;
};

} // namespace TinyEXIF

#endif // __TINYEXIF_H__
