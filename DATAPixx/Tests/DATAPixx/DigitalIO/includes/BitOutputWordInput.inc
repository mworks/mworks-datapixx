var bit0 = false {
    report ('bit0 = $bit0')
}

var bit1 = false {
    report ('bit1 = $bit1')
}

var bit2 = false {
    report ('bit2 = $bit2')
}

var bit3 = false {
    report ('bit3 = $bit3')
}

var word = 0 {
    report ('word = $word')
}


var index = 0

%define output_bits = [[0,0,0,0],
                       [1,0,0,0],
                       [0,1,0,0],
                       [0,0,1,0],
                       [0,0,0,1],
                       [1,1,1,1]]

%define update_bits ()
    index += 1
    bit0 = output_bits[index][0]
    bit1 = output_bits[index][1]
    bit2 = output_bits[index][2]
    bit3 = output_bits[index][3]
%end

%define wait_and_assert ()
    wait (1s)
    assert (word == (output_bits[index][0] * 1 +
                     output_bits[index][1] * 2 +
                     output_bits[index][2] * 4 +
                     output_bits[index][3] * 8))
%end


protocol {
    index = -1
    update_bits ()

    block (nsamples = 5) {
        word = -1

        report ('Starting device')
        start_io_device (datapixx)

        wait_and_assert ()

        update_bits ()

        wait_and_assert ()

        report ('Stopping device')
        stop_io_device (datapixx)
    }
}
