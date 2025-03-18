/***********************************************************************
FrameReader - Abstract base class to read color or depth frames from a
source.
Copyright (c) 2013-2022 Oliver Kreylos

This file is part of the Kinect 3D Video Capture Project (Kinect).

The Kinect 3D Video Capture Project is free software; you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

The Kinect 3D Video Capture Project is distributed in the hope that it
will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Kinect 3D Video Capture Project; if not, write to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
02111-1307 USA
***********************************************************************/

#ifndef KINECT_FRAMEREADER_INCLUDED
#define KINECT_FRAMEREADER_INCLUDED

#include <Kinect/Types.h>

/* Forward declarations: */
namespace Kinect {
class FrameBuffer;
}

namespace Kinect {

class FrameReader
	{
	/* Elements: */
	protected:
	Size size; // Width and height of returned frames
	
	/* Constructors and destructors: */
	public:
	virtual ~FrameReader(void);
	
	/* Methods: */
	const Size& getSize(void) const // Returns the frame size
		{
		return size;
		}
	unsigned int getSize(int dimension) const // Returns frame width or height
		{
		return size[dimension];
		}
	virtual FrameBuffer readNextFrame(void) =0; // Returns the next color or depth frame
	};

}

#endif
