function insideRegion = room_door_region_check(estimate,xmin,xmax,ymin,ymax)
if (estimate(1) >= xmin && estimate(1) <= xmax && ...
        estimate(2) >= ymin && estimate(2) <= ymax)
    insideRegion = true;
else
    insideRegion = false;
end