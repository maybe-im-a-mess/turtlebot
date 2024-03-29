MODEL_TEMPLATE_DIR ?= model_templates

#############################################################
vpath %.sdf $(MODEL_TEMPLATE_DIR)

all: \
	world_1_1.sdf \
	world_2_2.sdf \
	world_5_5.sdf

LABYRINTH_COMMON_PREREQ_MODELS := \
	wall/ \
	wall_White/ \
	wall_Red/

world_%.sdf: challenge.sdf labyrinth_%/
	mako-render \
		--var world_name=$(basename $@) \
		--var labyrinth_name=$(subst /, ,$(word 2,$^)) \
		--var robot_pose_zr=pi/2 \
		$< > $@

world_5_5.sdf: challenge.sdf labyrinth_5_5/
	mako-render \
		--var world_name=$(basename $@) \
		--var labyrinth_name=$(subst /, ,$(word 2,$^)) \
		--var robot_pose_zr=pi/2 \
		$< > $@
	sed -i 's|\(</world>\)|$(CHOCOLATE)\n\1|' $@

CHOCOLATE := <include><name>chocolate</name><uri>https://fuel.ignitionrobotics.org/1.0/GoogleResearch/models/KS_Chocolate_Cube_Box_Assortment_By_Neuhaus_2010_Ounces</uri><pose>0.5 4.5 0 0 0 0</pose></include>

labyrinth_1_1/ \
labyrinth_2_2/ \
labyrinth_5_5/: | $(LABYRINTH_COMMON_PREREQ_MODELS) maze/

maze/:
	# https://scipython.com/blog/making-a-maze/
	mkdir maze
	curl https://raw.githubusercontent.com/scipython/scipython-maths/master/maze/df_maze.py > $@/df_maze.py

# trigger makefiles of all models if one file in model_templates changes
%/: model_templates
	# Create $@ only if mako-render exists. Otherwise make does not remove $@ and
	# assumes that the model is made.
	which mako-render && echo mako-render is not available
	mkdir -p $@
	ln -srf gazebo_model.mk $@/Makefile
	$(MAKE) -C $@

clean:
	$(RM) -r labyrinth_*
	$(RM) -r $(LABYRINTH_COMMON_PREREQ_MODELS)
	$(RM) world_*.sdf
	$(RM) -r maze
