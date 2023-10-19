class ArucoTag:
    id = ''
    marker_size = 0

    def __init__(self, in_id='', in_marker_size=0):
        """
        Purpose: in_id->id, in_marker_size->maker_size
        """
        
        self.id = in_id
        self.marker_size = in_marker_size
    # end alternate constructor

    
class TagData:
    list_of_tags = []

    