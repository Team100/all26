from typing import overload, SupportsInt, SupportsFloat, Generator

class Point2D:
    """2D point containing an X & Y value"""

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, x: SupportsInt | SupportsFloat, y: SupportsInt | SupportsFloat) -> None: ...

    def __init__(self, x: SupportsInt | SupportsFloat = 0, y: SupportsInt | SupportsFloat = 0) -> None:
        self.x: SupportsInt | SupportsFloat = x
        self.y: SupportsInt | SupportsFloat = y

    def as_list(self) -> list[SupportsInt | SupportsFloat]:
        """
        Returns the point as a list [x,y]
        
        :return: A list containing the point's x & y position
        :rtype: list[SupportsInt | SupportsFloat]
        """
        return [self.x, self.y]

    def as_tuple(self) -> tuple[SupportsInt | SupportsFloat, SupportsInt | SupportsFloat]:
        """
        Returns the point as a tuple [x,y]
        
        :return: A tuple containing the point's x & y position
        :rtype: tuple[SupportsInt | SupportsFloat, SupportsInt | SupportsFloat]
        """
        return (self.x, self.y)

    def __iter__(self) -> Generator[SupportsInt | SupportsFloat, SupportsInt | SupportsFloat]:
        yield self.x
        yield self.y

    def __reversed__(self) -> Generator[SupportsInt | SupportsFloat, SupportsInt | SupportsFloat]:
        yield self.y
        yield self.x

    def __len__(self) -> int:
        return 2

    def __setitem__(self, ndx: int, data: SupportsInt | SupportsFloat) -> None:
        l = [self.x, self.y]
        l[ndx] = data
        self.x, self.y = l

    def __getitem__(self, ndx: int) -> SupportsInt | SupportsFloat:
        return (self.x, self.y)[ndx]
    
class Point3D:
    """3D point containing an X, Y & Z value"""

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, x: SupportsInt | SupportsFloat, y: SupportsInt | SupportsFloat, z: SupportsInt | SupportsFloat) -> None: ...

    def __init__(self, x: SupportsInt | SupportsFloat = 0, y: SupportsInt | SupportsFloat = 0, z: SupportsInt | SupportsFloat = 0) -> None:
        self.x: SupportsInt | SupportsFloat = x
        self.y: SupportsInt | SupportsFloat = y
        self.z: SupportsInt | SupportsFloat = z

    def as_list(self) -> list[SupportsInt | SupportsFloat]:
        """
        Returns the point as a list [x,y,z]
        
        :return: A list containing the point's X, Y, & Z position
        :rtype: list[SupportsInt | SupportsFloat]
        """
        return [self.x, self.y, self.z]

    def as_tuple(self) -> tuple[SupportsInt | SupportsFloat, SupportsInt | SupportsFloat, SupportsInt | SupportsFloat]:
        """
        Returns the point as a tuple [x,y,z]
        
        :return: A tuple containing the point's X, Y, & Z position
        :rtype: tuple[SupportsInt | SupportsFloat, SupportsInt | SupportsFloat, SupportsInt | SupportsFloat]
        """
        return (self.x, self.y, self.z)

    def __iter__(self) -> Generator[SupportsInt | SupportsFloat, SupportsInt | SupportsFloat]:
        yield self.x
        yield self.y
        yield self.z

    def __reversed__(self) -> Generator[SupportsInt | SupportsFloat, SupportsInt | SupportsFloat]:
        yield self.z
        yield self.y
        yield self.x

    def __len__(self) -> int:
        return 3

    def __setitem__(self, ndx: int, data: SupportsInt | SupportsFloat) -> None:
        l = [self.x, self.y, self.z]
        l[ndx] = data
        self.x, self.y, self.z = l

    def __getitem__(self, ndx: int) -> SupportsInt | SupportsFloat:
        return (self.x, self.y, self.z)[ndx]