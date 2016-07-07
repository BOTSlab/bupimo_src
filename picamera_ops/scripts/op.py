class Op:
    """Interface / abstract class for operations using the PiCamera"""

    def __init__(self, settings):
        # Should only be called by derived classes
        raise NotImplementedError

    def apply(self, image, image_debug):
        # Should only be called by derived classes
        raise NotImplementedError
