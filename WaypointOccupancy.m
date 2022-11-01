function z = WaypointOccupancy(xy)
z = getOccupancy(map,[xy(1),xy(2)]);
end