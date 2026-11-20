pico-8 cartridge // http://www.pico-8.com
version 41
__lua__
-- power-up screen of the core (display buffer contents before the first
-- frame, shown while no program runs); rendered into splash.hex on the
-- verilator model (sim/obj_dir/Vp8_soc --screen-hex)
function _draw()
 cls(1)
 rectfill(0,0,127,9,12)
 print("1retro p8",46,2,1)
 print("copy boot.rom to",32,44,7)
 print("games/p8/ and restart",22,52,7)
 print("or press f12 and choose",18,76,6)
 print("load compiled cart",28,84,6)
end
