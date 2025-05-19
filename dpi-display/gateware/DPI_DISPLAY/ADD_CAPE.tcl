puts "======== Add cape option: SPI ========"

#-------------------------------------------------------------------------------
# Import HDL source files
#-------------------------------------------------------------------------------
import_files -hdl_source "$cape_dir/HDL/spi.v"
import_files -hdl_source "$cape_dir/HDL/display.v"
import_files -hdl_source "$cape_dir/HDL/CAPE.v"

build_design_hierarchy

create_hdl_core -file $project_dir/hdl/CAPE.v -module {CAPE} -library {work} -package {}

#-------------------------------------------------------------------------------
# Build the Cape module
#-------------------------------------------------------------------------------
set sd_name ${top_level_name}

#-------------------------------------------------------------------------------
# Cape pins
#-------------------------------------------------------------------------------
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_11} -port_direction {IN}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_13} -port_direction {OUT}

sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_12} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_14} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_15} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_16} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_17} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_18} -port_direction {OUT}

sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_21} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_22} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_23} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_24} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_25} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_26} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_27} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_28} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_29} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_30} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_31} -port_direction {OUT}

sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_41} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P9_42} -port_direction {OUT}


sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_3} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_4} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_5} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_6} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_7} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_8} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_9} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_10} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_11} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_12} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_13} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_14} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_15} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_16} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_17} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_18} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_19} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_20} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_21} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_22} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_23} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_24} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_25} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_26} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_27} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_28} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_29} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_30} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_31} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_32} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_33} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_34} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_35} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_36} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_37} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_38} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_39} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_40} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_41} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_42} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_43} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_44} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_45} -port_direction {OUT}
sd_create_scalar_port -sd_name ${sd_name} -port_name {P8_46} -port_direction {OUT}

#-------------------------------------------------------------------------------
# Instantiate.
#-------------------------------------------------------------------------------

sd_instantiate_hdl_core -sd_name ${sd_name} -hdl_core_name {CAPE} -instance_name {CAPE}

#-------------------------------------------------------------------------------
# Connections.
#-------------------------------------------------------------------------------

# Clocks and resets
sd_connect_pins -sd_name ${sd_name} -pin_names {"CLOCKS_AND_RESETS:FIC_0_ACLK" "CAPE:ACLK"}
sd_connect_pins -sd_name ${sd_name} -pin_names {"CLOCKS_AND_RESETS:FIC_0_FABRIC_RESET_N" "CAPE:ARESETN" }

# GPIO
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:GPIO_2_M2F" "CAPE:GPIO_2_M2F" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:GPIO_2_F2M" "CAPE:GPIO_2_F2M" }

## Display
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_23" "CAPE:VSYNC" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_25" "CAPE:HSYNC" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_27" "CAPE:B2" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_29" "CAPE:R3" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_31" "CAPE:G2" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_33" "CAPE:B7" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_35" "CAPE:G3" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_37" "CAPE:PCLK" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_39" "CAPE:B3" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_41" "CAPE:B4" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_43" "CAPE:G5" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_45" "CAPE:R5" }

sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_28" "CAPE:G6" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_30" "CAPE:G7" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_32" "CAPE:R4" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_34" "CAPE:B6" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_36" "CAPE:B5" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_38" "CAPE:DATA_ENABLE" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_40" "CAPE:G4" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_42" "CAPE:R2" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_44" "CAPE:R6" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_46" "CAPE:R7" }

## AXI4
### Read Address Channel
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARADDR" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARADDR" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARID" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARID" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARLEN" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARLEN" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARBURST" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARBURST" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARSIZE" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARSIZE" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARLOCK" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARLOCK" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARCACHE" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARCACHE" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARPROT" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARPROT" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARQOS" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARQOS" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARVALID" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARVALID" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARREADY" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARREADY" }

### AXI4 Read Data Channel
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RDATA" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RDATA" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RID" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RID" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RRESP" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RRESP" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RLAST" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RLAST" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RVALID" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RVALID" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RREADY" "CAPE:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RREADY" }

### Tie down write channels
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_AWVALID} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_WVALID} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_BREADY} -value {GND}

### FIC_1 Read Address Channel
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARADDR" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARADDR" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARID" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARID" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARLEN" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARLEN" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARBURST" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARBURST" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARSIZE" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARSIZE" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARLOCK" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARLOCK" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARCACHE" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARCACHE" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARPROT" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARPROT" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARQOS" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARQOS" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARVALID" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARVALID" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARREADY" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARREADY" }

### FIC_1 Read Data Channel
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RDATA" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RDATA" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RID" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RID" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RRESP" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RRESP" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RLAST" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RLAST" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RVALID" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RVALID" }
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RREADY" "CAPE:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RREADY" }

### FIC_1 Tie down write channels
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_AWVALID} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_WVALID} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_BREADY} -value {GND}

## Debug connections
# sd_connect_pins -sd_name ${sd_name} -pin_names {"P9_21" "CAPE:SPI_MOSI" }
# sd_connect_pins -sd_name ${sd_name} -pin_names {"P9_23" "CAPE:SPI_CLK" }
# sd_connect_pins -sd_name ${sd_name} -pin_names {"P9_25" "CAPE:SPI_ENABLE" }

# sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_39" "BVF_RISCV_SUBSYSTEM:GPIO_2_M2F[20:20]" }

# sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_37" "BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_ARVALID" }
# sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_35" "BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RREADY" }
# sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_33" "BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RVALID" }
# # sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_31" "BVF_RISCV_SUBSYSTEM:FIC_0_AXI4_TARGET_FIC_0_AXI4_S_RLAST"}
# sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_31" "CLOCKS_AND_RESETS:FIC_0_ACLK"}

# sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_29" "BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_ARVALID" }
# sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_27" "BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RREADY" }
# sd_connect_pins -sd_name ${sd_name} -pin_names {"P8_25" "BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_TARGET_FIC_1_AXI4_S_RVALID" }


# UART
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:MMUART_4_TXD" "P9_13"}
sd_connect_pins -sd_name ${sd_name} -pin_names {"BVF_RISCV_SUBSYSTEM:MMUART_4_RXD" "P9_11"}

#-------------------------------------------------------------------------------
# Unused pins

sd_mark_pins_unused -sd_name ${sd_name} -pin_names {BVF_RISCV_SUBSYSTEM:GPIO_2_OE_M2F}
sd_mark_pins_unused -sd_name ${sd_name} -pin_names {BVF_RISCV_SUBSYSTEM:FIC_1_AXI4_INITIATOR}
sd_mark_pins_unused -sd_name ${sd_name} -pin_names {BVF_RISCV_SUBSYSTEM:CAPE_APB_MTARGET}

# Connect the unused cape pins to GND
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_12} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_14} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_15} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_16} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_17} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_18} -value {GND}

sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_21} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_22} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_23} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_24} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_25} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_26} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_27} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_28} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_29} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_30} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_31} -value {GND}

sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_41} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P9_42} -value {GND}


sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_3} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_4} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_5} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_6} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_7} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_8} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_9} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_10} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_11} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_12} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_13} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_14} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_15} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_16} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_17} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_18} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_19} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_20} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_21} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_22} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_23} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_24} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_25} -value {GND}
sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_26} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_27} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_28} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_29} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_30} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_31} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_32} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_33} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_34} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_35} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_36} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_37} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_38} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_39} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_40} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_41} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_42} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_43} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_44} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_45} -value {GND}
# sd_connect_pins_to_constant -sd_name ${sd_name} -pin_names {P8_46} -value {GND}
