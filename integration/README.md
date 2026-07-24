# integration/ — Combined Pipeline

This folder will contain the code that connects YOLO detection output 
to OMX robot arm control — the final combined pipeline.

## Files (to be added)
- `main_pipeline.py` — calls YOLO's detect_and_select() function, 
  passes the result to OMX's pick_and_place() function, and runs the full loop

## Purpose
This is where the YOLO team's and OMX team's code actually meet and work together. 
Both teams should review changes here since it affects both sides.

## Status
Not yet started — will be built once both YOLO detection and OMX pick-and-place 
functions are individually working 