/*! \file
	\section FileCopyright Copyright Notice
	This is free and unencumbered software released into the public domain.

	Anyone is free to copy, modify, publish, use, compile, sell, or
	distribute this software, either in source code form or as a compiled
	binary, for any purpose, commercial or non-commercial, and by any
	means.

	In jurisdictions that recognize copyright laws, the author or authors
	of this software dedicate any and all copyright interest in the
	software to the public domain. We make this dedication for the benefit
	of the public at large and to the detriment of our heirs and
	successors. We intend this dedication to be an overt act of
	relinquishment in perpetuity of all present and future rights to this
	software under copyright law.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
	EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
	MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
	IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
	ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
	OTHER DEALINGS IN THE SOFTWARE.
*/

#include "xsens_SDK/timecodedatagram.h"

/*! \class TimeCodeDatagram
	\brief a Time Code datagram (type 0x25)

	Information about each segment is sent as follows.

	4 bytes String size (12 byte)
	12-byte timecode string: HH:MM:SS.mmm

	Total: 16 bytes per segment
*/

/*! Constructor */
TimeCodeDatagram::TimeCodeDatagram()
	: Datagram()
{
	setType(SPTimeCode);
}

/*! Destructor */
TimeCodeDatagram::~TimeCodeDatagram()
{
}

/*! Deserialize the data from \a arr
	\sa serializeData
*/
void TimeCodeDatagram::deserializeData(Streamer &inputStreamer)
{
	Streamer* streamer = &inputStreamer;

	int32_t stringSize = 0;
	streamer->read(stringSize);

	std::string str;
	streamer->read(str, 12);
	int h,m,s,n;

	sscanf(str.c_str(), "%d:%d:%d.%d", &h, &m, &s, &n);
	m_hour = h;
	m_minute = m;
	m_second = s;
	m_nano = 1000000*n;
}

/*! Print Time Code datagram in a formated why
*/
void TimeCodeDatagram::printData() const
{
	std::cout << "*********************** DATA CONTENT ***********************" <<  std::endl <<  std::endl;
	printf("%d:%d:%d.%lu", m_hour, m_minute, m_second, m_nano);
	std::cout << std::endl;
}

/*! Print Time Code datagram in CSV format
*/
void TimeCodeDatagram::printCSVData() const
{
	printf("%d,%d,%d,%lu", m_hour, m_minute, m_second, m_nano);
	std::cout << std::endl;
}

/*! Get the datagram value
*/
TimeCode TimeCodeDatagram::getData()
{
  TimeCode time;
  time.m_hour = m_hour;
  time.m_minute = m_minute;
  time.m_second = m_second;
  time.m_nano = m_nano;
  return time;
}
