# goethe
Accurate, low volume robotic color matching and mixing platform for medium to high viscosity paint

## What is this project?
The idea of this project is to create a physical version of the colorpicker/eyedropper or
palette creators you might see in a digital image-editing software like photoshop, but for paint
(and in theory other physical liquid mediums).

## What do we have so far?
![v1 prototype](/images/11_15_2020.bmp)

A (grossly over-engineered, but still usable) prototype with XY-axis control, and a raisable and controllable syringe for picking up and putting down paints (and potentially stirring). Currently based on the Arduino UNO, will be moving to Arduino Mega soon for development, and considering out other AVR platforms for longer term after initial prototyping. The prototype has everything it needs for movement and controls, and has basic movement (non-control looped) for the carriage and the syringe already validated. Currently missing a platform to hold the paint and water reservoirs, and a sliding platform to hold the user's palette. Also not wired up for the color sensor yet, and I haven't made the CAD designs for the sensor pen/eyedropper (key thing here is that we want the tool to reject ambient light for color consistency across usage environments). A lot of the control code is written (but unvalidated), and is also pretty rough but usable. Actual wrappers for using the system for color mixing is still WIP.

*Important note: The repo right now is still just being used as a place for me to dump and track things as I continue development. As such, files may be disorganized or incorrectly labeled, code can be expected to still be buggy or unvalidated, and this repo may not contain all resources associated with the project, such as notes or a BOM for the non-3d-printed parts. You're free to try to build your own, but **be advised that some parts may be missing or unfinished**. If the results of this project turn out good, I'll go back through and do some housekeeping/updates later to make it easier to build your own.*

## Next Steps
Well, first I want to finish this prototype. Then, I want to mock a prototype that uses peristaltic pumps and a strain guage/scale as a 
potentially faster, cheaper, simpler design, and compare color accuracy and precision across the two designs. (This design was actually one of
the original concepts, although I rejected it bc I liked the elegance of the V1 design (only one extruder), and thought it may have issues with
accuracy. After seeing industrial paint mixing machines and that they work with a similar pump design, I figured it's worth revisiting (although it's
possible that the color accuracy of this design at low volume in contrast to paint can volume is just not very good). This design technically uses
more motors (and potentially even more if we want to add powered XY axis support for comparable workspace instead of static nozzle placement),
but the mixing process steps are likely simpler in this approach, and worth considering given the relatively cheap cost of peristaltic pumps.