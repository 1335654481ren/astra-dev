#include <stdio.h>
#include <string.h>
#include <xdo.h>

int main()
{
    Window *list = NULL;
    xdo_t *my_xdo = xdo_new(NULL);
    xdo_search_t search;
    unsigned int nwindows;
    unsigned int i;

    memset(&search, 0, sizeof(xdo_search_t));

    /* setup search */
    search.require = SEARCH_ANY;
    search.max_depth = -1;
    search.searchmask |= (SEARCH_CLASSNAME | SEARCH_ONLYVISIBLE);
    search.only_visible = True;
    char *winname = "gedit";
    search.winclassname= winname;

    /* do search */
    xdo_search_windows(my_xdo,&search,&list,&nwindows);

    /* print results */
    printf("number of windows: %d\n", nwindows);
    unsigned char * name;
    int size;
    int type;
    for(int i=0; i<nwindows; i++ ){
        xdo_get_window_name(my_xdo,list[i],&name,&size,&type);
        printf("%d: %s\n",i, name);
    }

    /* Send keypresses to first gedit in list */


    for(int i=0; i<1000; i++) {
        xdo_activate_window(my_xdo, list[0]);
        //xdo_move_mouse(my_xdo, i, i, screen);
        xdo_move_mouse_relative(my_xdo, i, i);
         sleep(1);
        // xdo_enter_text_window(my_xdo,CURRENTWINDOW,"HELLO",1000);
        // sleep(1);
        // xdo_enter_text_window(my_xdo,CURRENTWINDOW,"WORLD",1000);
        // sleep(1);
        // xdo_send_keysequence_window(my_xdo,CURRENTWINDOW,"ctrl+a",1000);
        // sleep(1);
        // xdo_send_keysequence_window(my_xdo,CURRENTWINDOW,"Delete",1000);
        // sleep(3);
    }
    return 0;
}