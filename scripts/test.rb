#!/usr/bin/env ruby

require 'orocos'
require 'rock/bundle'
require 'readline'
require 'vizkit'

include Orocos

# Initialize bundles to find the configurations for the packages
Bundles.initialize

Orocos.conf.load_dir('../../../../bundles/rover/config/orogen')

# Execute the task
Orocos::Process.run 'platform_driver::Exoter' => 'platfrom_driver' do

    # Configure
    platform_driver = Orocos.name_service.get 'platform_driver'
    Orocos.conf.apply(platform_driver, ['arm'], :override => true)
    platform_driver.configure

    # Log
    #Orocos.log_all_ports
    #platform_driver.log_all_ports
    #pancam_panorama.log_all_ports

    # Connect

    # Start
    platform_driver.start

    Readline::readline("Press Enter to exit\n") do
    end
end
