#!/usr/bin/perl

# This stub turns on LD64 for us. Right now this is all it does, but we
# could expand it later. It used to do more, and it might do more in the
# future, so the hooks in the Python build tools are maintained. Cameron Kaiser

$ENV{'LD64'} = '1';
exec(@ARGV); # fall through and link normally
die("could not link to $ARGV[0]: $!\n"); 

