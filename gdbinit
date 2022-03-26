set $_exitcode = -1
define hookpost-next
backtrace
end
run
if $_exitcode != -1
    quit
end