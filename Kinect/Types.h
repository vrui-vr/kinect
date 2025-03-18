/***********************************************************************
Types - Declaration of common types used throughout the Kinect 3D Video
Capture Project.
Copyright (c) 2022 Oliver Kreylos

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

#ifndef KINECT_TYPES_INCLUDED
#define KINECT_TYPES_INCLUDED

#include <Misc/Offset.h>
#include <Misc/Size.h>
#include <Misc/Rect.h>

namespace Kinect {

typedef Misc::Offset<2> Offset; // Type for image or frame offsets and pixel positions
typedef Misc::Size<2> Size; // Type for image or frame sizes
typedef Misc::Rect<2> Rect; // Type for image or frame rectangles

}

#endif
