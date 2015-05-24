function ang_a = angular_acceleration()
    ang_a = revI*(cross(-omega(:, index), I*omega(:, index)))+revI*thrust;
end